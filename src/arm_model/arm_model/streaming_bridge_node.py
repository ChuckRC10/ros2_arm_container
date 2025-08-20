# streaming_bridge_node.py - The "Consumer" 🤖
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
import multiprocessing as mp
import sys
import atexit
import numpy as np
import time

# ‼️ IMPORTANT: Replace with the ACTUAL path to your script's folder
sys.path.append('//ros2_ws//src//arm_model//scripts//pygame_make_angles')
import test_sin

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
        
        # Subscriber for joint positions
        self.odom_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.odom_callback,
            10)

        # Publisher for the joint trajectory controller
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # ‼️ IMPORTANT: Make sure these joint names EXACTLY match your controller's YAML file
        self.joint_names = ['joint1', 'joint2', 'joint3']
        
        # Create a queue for communication and start the producer process
        self.queue = mp.Queue()
        self.producer_process = mp.Process(target=test_sin.calculate_angles_loop, args=(self.queue,))
        self.producer_process.start()

    def key_callback(self, msg):
        self.get_logger().info('I keyed: "%d"' % msg.data)
        
    def odom_callback(self, msg):
        self.joint_positions = msg.position
        formatted_positions = [f'{p:.3f}' for p in self.joint_positions]
        self.get_logger().info(f'I jointed: {formatted_positions}')
        time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    streaming_bridge_node = StreamingBridgeNode()
    rclpy.spin(streaming_bridge_node)
    # The cleanup function will be called automatically on shutdown (e.g., Ctrl+C)
    streaming_bridge_node.destroy_node()
    rclpy.shutdown()

#def get_2d_arm_vectors(lengths, angles):
    # TODO: calculate 2d vectors of arms

#def rotate_arm_vectors(angle, vectors_2d):
    # TODO: calculate 3d vectors of arms due to base rotation

def get_end_effector(vectors_3d):
    return np.sum(vectors_3d)

# TODO: figure out how to translate original arm inverse kinematics code to this system

if __name__ == '__main__':
    main()