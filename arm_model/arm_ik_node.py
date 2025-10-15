import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
import jax.numpy as jnp
from .kinematics import armClass, inv_kinematics_least_sqr
from math import isfinite

class ArmIKNode(Node):
    def __init__(self):
        super().__init__('arm_ik_node')

        # Subscriber for keystroke node
        self.teleop_subscription = self.create_subscription(
            Vector3,
            '/arm_controller/cmd_vel',
            self.velocity_callback,
            10)

        # initialize movement delta to zero so timer can run before any Twist arrives
        self.target_velocity_vec = jnp.array([0.0, 0.0, 0.0])

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
        timer_period = 0.02 # 50 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # initialize arm class
        self.joint_positions = [0.0, 0.0, 0.0]
        self.arm = armClass(jnp.array([0.5, 0.5]), jnp.array(self.joint_positions))

        self.wanted_pos = jnp.array([0, 0, 1])
        self.dampingConstant = .03

        self.get_logger().info("Arm IK Node started.")

    def velocity_callback(self, msg):
        self.target_velocity_vec = jnp.array([msg.x, msg.y, msg.z])

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

        # store as numpy array
        self.movementDeltaVec = np.array([lx, ly, lz])

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
        dt = self.timer.timer_period_ns / 1e9
        position_delta = self.target_velocity_vec * dt

        current_pose = self.arm.get_end_effector(current_arm_angles)
        target_pose = current_pose + position_delta

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
        point.time_from_start.nanosec = int(dt * 1e9)
        traj_msg.points.append(point)

        if len(new_position) == len(self.joint_names) and all(isfinite(p) for p in new_position):
            self.publisher_.publish(traj_msg)
        else:
            self.get_logger().warning(f"Not publishing invalid trajectory: {new_position}")

def main(args=None):
    rclpy.init(args=args)
    arm_ik_node = ArmIKNode()
    rclpy.spin(arm_ik_node)
    # The cleanup function will be called automatically on shutdown (e.g., Ctrl+C)
    arm_ik_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()