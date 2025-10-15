import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np
from .kinematics import armClass, inv_kinematics_least_sqr
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
        self.movementDeltaVec = np.array([0.0, 0.0, 0.0])

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
        self.arm = armClass(np.array([0.5, 0.5]), np.array(self.joint_positions))

        self.wanted_pos = np.array([0, 0, 1])
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

        # store as numpy array
        self.movementDeltaVec = np.array([lx, ly, lz])

    def joint_state_callback(self, msg):
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        positions = [float(name_to_pos[n]) for n in self.joint_names if n in name_to_pos]
        if len(positions) == len(self.joint_names):
            self.joint_positions = positions

    def timer_callback(self):
        # calculate joint positions
        current_arm_angles = np.array(self.joint_positions)
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
        self.movementDeltaVec = np.array([0,0,0])

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