import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int32

class teleopInputNode(Node):
    def __init__(self):
        super().__init__('teleop_input_node')


        # Declare and get parameters
        self.declare_parameter('move_speed', 0.8)  # meters/second
        self.declare_parameter('key_timeout', 0.5) # seconds
        self.move_speed = self.get_parameter('move_speed').get_parameter_value().double_value
        self.key_timeout = self.get_parameter('key_timeout').get_parameter_value().double_value

        # Mapping from keycode to a velocity vector
        self.key_to_velocity = {
            68: (self.move_speed, 0.0, 0.0),   # D: +X
            65: (-self.move_speed, 0.0, 0.0),  # A: -X
            69: (0.0, self.move_speed, 0.0),   # E: +Y
            81: (0.0, -self.move_speed, 0.0),  # Q: -Y
            87: (0.0, 0.0, self.move_speed),   # W: +Z
            83: (0.0, 0.0, -self.move_speed),  # S: -Z
        }

        # State variables
        self.target_velocity = [0.0, 0.0, 0.0]
        self._last_key_time = self.get_clock().now()

        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Vector3, '/arm_controller/cmd_vel', 10)

        # Subscriber for keystroke node
        self.key_subscription = self.create_subscription(
            Int32, '/keyboard/keypress', self.key_callback, 10)

        # create timer to run calculation and publish
        timer_period = 0.1 # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("Teleop Input Node started.")

    def key_callback(self, msg):
        # Get the velocity vector for the pressed key, or (0,0,0) if it's not a move key
        self.target_velocity = self.key_to_velocity.get(msg.data, [0.0, 0.0, 0.0])
        self._last_key_time = self.get_clock().now()
        self.get_logger().info(f"Key {msg.data} received, setting velocity to {self.target_velocity}")

    def timer_callback(self):
        # Check for key release timeout
        if (self.get_clock().now() - self._last_key_time).nanoseconds / 1e9 > self.key_timeout:
            self.target_velocity = [0.0, 0.0, 0.0] # Set velocity to zero if no key pressed recently

        # Publish the current target velocity
        msg = Vector3()
        msg.x, msg.y, msg.z = self.target_velocity
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    teleop_input_node = teleopInputNode()
    rclpy.spin(teleop_input_node)
    # The cleanup function will be called automatically on shutdown (e.g., Ctrl+C)
    teleop_input_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()