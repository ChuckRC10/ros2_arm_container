# streaming_bridge_node.py - The "Consumer" 🤖
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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
        
        # ‼️ IMPORTANT: Make sure these joint names EXACTLY match your controller's YAML file
        self.joint_names = ['joint1', 'joint2', 'joint3']
        
        # Create a queue for communication and start the producer process
        self.queue = mp.Queue()
        self.producer_process = mp.Process(target=test_sin.calculate_angles_loop, args=(self.queue,))
        self.producer_process.start()
        
        # Register a cleanup function to run on exit
        atexit.register(self.cleanup)

        # Create a high-frequency timer to check the queue
        timer_period = 0.01  # 100Hz, faster than the producer
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        try:
            # Get data from the queue without blocking
            angles = self.queue.get_nowait()
            self.get_logger().info(f'Received angles: {[f"{a:.2f}" for a in angles]}', throttle_duration_sec=1.0)
            
            # Create and publish the trajectory message
            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = [float(angle) for angle in angles]
            # Set a small duration for a smooth transition
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 150000000 # 0.15 seconds
            
            traj_msg.points.append(point)
            self.publisher_.publish(traj_msg)

        except mp.queues.Empty:
            # This is expected if the consumer is faster than the producer
            pass

    def cleanup(self):
        """Ensure the producer process is terminated when the node is shut down."""
        self.get_logger().info("Shutting down, terminating producer process...")
        if self.producer_process.is_alive():
            self.producer_process.terminate()
            self.producer_process.join()
        self.get_logger().info("Producer process terminated.")

def main(args=None):
    rclpy.init(args=args)
    streaming_bridge_node = StreamingBridgeNode()
    rclpy.spin(streaming_bridge_node)
    # The cleanup function will be called automatically on shutdown (e.g., Ctrl+C)
    streaming_bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()