import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import jax
import jax.numpy as jnp
from jax.numpy import sin, cos
from math import isfinite

class ArmIKNode(Node):
    def __init__(self):
        super().__init__('arm_ik_node')
        self.get_logger().info("Arm IK Node started.")

        # Subscriber for keystroke node
        self.teleop_subscription = self.create_subscription(
            Twist,
            '/wntd_delta_arm_pos',
            self.teleop_callback,
            10)
        
        # initialize movement delta to zero so timer can run before any Twist arrives
        self.movementDeltaVec = jnp.array([0.0, 0.0, 0.0])

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
        self.arm = armClass(jnp.array([0.5, 0.5]), jnp.array(self.joint_positions))

        self.wanted_pos = jnp.array([0, 0, 1])
        self.dampingConstant = .03

        self._last_logged_movement = None

    def teleop_callback(self, msg):
        # Log received linear components (guarded formatting)
        try:
            lx = _safe(msg.linear.x)
            ly = _safe(msg.linear.y)
            lz = _safe(msg.linear.z)
            vec = (lx, ly, lz)  # tuple of floats

            if self._last_logged_movement is None or any(abs(a-b) > 1e-6 for a, b in zip(vec, self._last_logged_movement)):
                self.get_logger().info(f"I receive twist linear=({vec[0]:.4f}, {vec[1]:.4f}, {vec[2]:.4f})")
                self._last_logged_movement = vec
        except Exception:
            # fallback logging if msg structure is unexpected
            self.get_logger().info(f"I receive twist (unexpected format): {msg}")
            lx = ly = lz = 0.0

        # store as jnp.array so later arithmetic with jnp arrays is consistent
        self.movementDeltaVec = jnp.array([lx, ly, lz])

    def joint_state_callback(self, msg):
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        positions = [float(name_to_pos[n]) for n in self.joint_names if n in name_to_pos]
        if len(positions) == len(self.joint_names):
            self.joint_positions = positions

    def timer_callback(self):
        # calculate joint positions
        current_arm_angles = jnp.array(self.joint_positions)
        self.arm.arm_angles = current_arm_angles

        # update wanted position
        current_pose = self.arm.get_end_effector(current_arm_angles)
        target_pose = current_pose + self.movementDeltaVec

        # get change in arm angles
        jacobian = self.arm.get_jacobian()
        error = target_pose - current_pose 
        delta_q = inv_kinematics_least_sqr(jacobian, error, self.dampingConstant)

        # update arm angles
        target_arm_angles = self.arm.arm_angles + delta_q

        # send joint positions to robot
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        new_position = target_arm_angles.tolist()
        point.positions = new_position
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 150000000 # 0.15 seconds

        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)

        # reset movement vector after use
        self.movementDeltaVec = jnp.array([0,0,0])

class armClass:
    def __init__(self, arm_lengths:jnp.array, arm_angles:jnp.array):
        self.arm_lengths = arm_lengths
        self.arm_angles = arm_angles

    def getArmVectors(self, angles) -> jnp.array:
        vectors_2d = self.__get2dArmVectors__(angles)
        vectors_3d = self.__rotate_arm_vectors__(vectors_2d, angles)

        return vectors_3d

    def __get2dArmVectors__(self, angles):
        angles2d = angles[1:]
        globalAngles = jnp.cumsum(angles2d)

        # calculate vector coordinates
        xArray = self.arm_lengths * sin(globalAngles)
        yArray = self.arm_lengths * cos(globalAngles)
    
        armVectorArray = jnp.array([xArray, yArray]).T
        return armVectorArray

    def __rotate_arm_vectors__(self, vectors_2d, angles):
        base_angle = angles[0]
        cos_theta = cos(base_angle)
        sin_theta = sin(base_angle)
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
    

def inv_kinematics_least_sqr(Jacobian: jnp.array, error: jnp.array, damping_c: float) -> list:
    """Calculate delta q, damped least-squares"""
    eye_size = Jacobian.shape[1]
    damping_matrix = jnp.linalg.inv(Jacobian.T @ Jacobian + damping_c**2 * jnp.eye(eye_size))
    delta_q = damping_matrix @ Jacobian.T @ error
    return delta_q

def _safe(v, default=0.0):
    try:
        if not isfinite(v):
            return default
        return float(v)
    except Exception:
        return default
    
def main(args=None):
    rclpy.init(args=args)
    arm_ik_node = ArmIKNode()
    rclpy.spin(arm_ik_node)
    # The cleanup function will be called automatically on shutdown (e.g., Ctrl+C)
    arm_ik_node.destroy_node()
    rclpy.shutdown()

# TODO: figure out how to translate original arm inverse kinematics code to this system

if __name__ == '__main__':
    main()