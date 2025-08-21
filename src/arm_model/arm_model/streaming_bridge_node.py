# streaming_bridge_node.py - The "Consumer"
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
import sys
import jax
import jax.numpy as jnp
from jax.numpy import sin, cos

class StreamingBridgeNode(Node):
    def __init__(self):
        super().__init__('streaming_bridge_node')
        self.get_logger().info("Streaming Bridge Node started.")

        # Subscriber for keystroke node
        self.key_subscription = self.create_subscription(
            Int32,
            '/keyboard/keypress',
            self.key_callback,
            10)
        
        self.current_key = 0

        # Subscriber for joint positions
        self.state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

        # Publisher for the joint trajectory controller
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Joint names from yaml file
        self.joint_names = ['joint1', 'joint2', 'joint3']

        # create timer to run calculation and publish
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # initialize arm class
        self.joint_positions = [0.0, 0.0, 0.0]
        self.arm = armClass(jnp.array([0.5, 0.5]), jnp.array([0, 0]), 0)

        self.wanted_pos = jnp.array([0, 0, 1])
        self.dampingConstant = .1
        self.maxPositionDelta = .005 

    def key_callback(self, msg):
        self.get_logger().info('I keyed: "%d"' % msg.data)
        self.current_key = msg.data
        
    def joint_state_callback(self, msg):
        self.joint_positions = msg.position
        formatted_positions = [f'{p:.3f}' for p in self.joint_positions]

    def timer_callback(self):
        # calculate joint positions
        current_base_angle = self.joint_positions[0]
        current_arm_angles = jnp.array(self.joint_positions[1:])
        self.arm.arm_angles = current_arm_angles
        self.arm.base_angle = current_base_angle

        # get info to move arm
        pressed_key = self.current_key
        movementDeltaVector = get_movement(pressed_key, self.maxPositionDelta)

        # update wanted position
        self.wanted_pos = self.wanted_pos - movementDeltaVector

        # get change in arm angles
        jacobian = self.arm.get_jacobian()
        error = self.arm.get_error(self.wanted_pos)
        delta_q = inv_kinematics_least_sqr(jacobian, error, self.dampingConstant)

        # update arm angles
        target_arm_angles = self.arm.arm_angles + delta_q

        arm_angles = self.arm.arm_angles.tolist()
        arm_angles.insert(0, self.arm.base_angle)

        # send joint positions to robot
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        new_position = [current_base_angle] + target_arm_angles.tolist()
        point.positions = new_position
        # self.get_logger().info(f'new position: {new_position}')
        # self.get_logger().info(f'error: {error}')
        # self.get_logger().info(f'jacobian: {jacobian}')
        # self.get_logger().info(f'position: {self.arm.get_end_effector(self.arm.arm_angles)}')
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 150000000 # 0.15 seconds

        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)

class armClass:
    def __init__(self, arm_lengths:jnp.array, arm_angles:jnp.array, base_angle):
        self.arm_lengths = arm_lengths
        self.arm_angles = arm_angles
        self.base_angle = base_angle

    def getArmVectors(self, angles) -> jnp.array:
        vectors_2d = self.__get2dArmVectors__(angles)
        vectors_3d = self.__rotate_arm_vectors__(vectors_2d)

        return vectors_3d

    def __get2dArmVectors__(self, angles):
        globalAngles = jnp.cumsum(angles)

        # calculate vector coordinates
        xArray = self.arm_lengths * sin(globalAngles)
        yArray = self.arm_lengths * -cos(globalAngles)
    
        armVectorArray = jnp.array([xArray, yArray]).T
        return armVectorArray

    def __rotate_arm_vectors__(self, vectors_2d):
        cos_theta = cos(self.base_angle)
        sin_theta = sin(self.base_angle)
        Rz = jnp.array([
            [cos_theta, -sin_theta, 0],
            [sin_theta,  cos_theta, 0],
            [0, 0, 1]
        ])

        # Get the x and z components from the 2D vectors
        x_coords = vectors_2d[:, 0]
        z_coords = vectors_2d[:, 1]

        # Create a column of zeros for the new y-coordinate
        y_coords = jnp.zeros(self.arm_lengths.shape[0])

        vectors_3d_flat = jnp.column_stack([x_coords, y_coords, z_coords])
            
        # We transpose the rotation matrix to correctly rotate the vectors.
        return (Rz @ vectors_3d_flat.T).T

    def get_end_effector(self, angles) -> jnp.array:
        armVectorArray = self.getArmVectors(angles)
        armEndVector = jnp.sum(armVectorArray, axis=0)
        return armEndVector
    
    def get_jacobian(self) -> jnp.array:
        J = jax.jacrev(lambda angles: self.get_end_effector(angles))
        return J(self.arm_angles)
    
    def get_error(self, wntd_pos: jnp.array) -> jnp.array:
        arm_pos = self.get_end_effector(self.arm_angles)
        error = wntd_pos - arm_pos
        return error

def inv_kinematics_least_sqr(Jacobian: jnp.array, error: jnp.array, damping_c: float) -> list:
    """Calculate delta q, damped least-squares"""
    eye_size = Jacobian.shape[1]
    damping_matrix = jnp.linalg.inv(Jacobian.T @ Jacobian + damping_c**2 * jnp.eye(eye_size))
    delta_q = damping_matrix @ Jacobian.T @ error
    return delta_q

def get_movement(pressed_key, maxPositionDelta) -> jnp.array:
    '''
    returns how far the pointer will move on the screen based on key inputs
    '''
    xMovement = 0
    zMovement = 0
    # get user input
    if pressed_key == 87:
        zMovement = maxPositionDelta
    if pressed_key == 83:
        zMovement = -maxPositionDelta
    if pressed_key == 65:
        xMovement = maxPositionDelta
    if pressed_key == 68:
        xMovement = -maxPositionDelta

    movementDeltaVector = jnp.array([xMovement, 0, zMovement])
    return movementDeltaVector

def main(args=None):
    rclpy.init(args=args)
    streaming_bridge_node = StreamingBridgeNode()
    rclpy.spin(streaming_bridge_node)
    # The cleanup function will be called automatically on shutdown (e.g., Ctrl+C)
    streaming_bridge_node.destroy_node()
    rclpy.shutdown()

# TODO: figure out how to translate original arm inverse kinematics code to this system

if __name__ == '__main__':
    main()