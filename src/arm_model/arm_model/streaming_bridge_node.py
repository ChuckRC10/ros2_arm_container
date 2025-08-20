# streaming_bridge_node.py - The "Consumer" 🤖
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int32
import multiprocessing as mp
import sys
import atexit

# ‼️ IMPORTANT: Replace with the ACTUAL path to your script's folder
sys.path.append('//ros2_ws//src//arm_model//scripts//pygame_make_angles')
import test_sin

class StreamingBridgeNode(Node):
    def __init__(self):
        super().__init__('streaming_bridge_node')
        self.get_logger().info("Streaming Bridge Node started.")

        # Publisher for the joint trajectory controller
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Subscriber for keystroke node
        self.subscription = self.create_subscription(
            Int32,
            '/keyboard/keypress',
            self.listener_callback,
            10)
        self.subscription
        
        # ‼️ IMPORTANT: Make sure these joint names EXACTLY match your controller's YAML file
        self.joint_names = ['joint1', 'joint2', 'joint3']
        
        # Create a queue for communication and start the producer process
        self.queue = mp.Queue()
        self.producer_process = mp.Process(target=test_sin.calculate_angles_loop, args=(self.queue,))
        self.producer_process.start()

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%d"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    streaming_bridge_node = StreamingBridgeNode()
    rclpy.spin(streaming_bridge_node)
    # The cleanup function will be called automatically on shutdown (e.g., Ctrl+C)
    streaming_bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()