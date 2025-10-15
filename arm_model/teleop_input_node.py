import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int32

class teleopInputNode(Node):
    def __init__(self):
        super().__init__('teleop_input_node')
        self.get_logger().info("Teleop Input Node started.")

        # Subscriber for keystroke node
        self.key_subscription = self.create_subscription(
            Int32,
            '/keyboard/keypress',
            self.key_callback,
            10)
        
        self.current_key = 0
        self.maxPositionDelta = 0.005 # max movement in meters per cycle I think?

        # Publisher for the joint trajectory controller
        self.publisher_ = self.create_publisher(Twist, '/wntd_delta_arm_pos', 10)
        self.joint_names = ['joint1', 'joint2', 'joint3']

        # create timer to run calculation and publish
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def key_callback(self, msg):
        self.get_logger().info('I keyed: "%d"' % msg.data)
        self.current_key = msg.data

    def timer_callback(self):
        movement_delta_vec = get_movement(self.current_key, self.maxPositionDelta)

        # only publish when there is meaningful movement
        if any(abs(v) > 1e-9 for v in movement_delta_vec):
            twist_msg = Twist()
            twist_msg.linear = Vector3(x=movement_delta_vec[0], y=movement_delta_vec[1], z=movement_delta_vec[2])
            twist_msg.angular = Vector3()
            self.publisher_.publish(twist_msg)
            self.get_logger().info('Publishing movement delta: x: %.4f, y: %.4f, z: %.4f' %
                                   (movement_delta_vec[0], movement_delta_vec[1], movement_delta_vec[2]))
        else:
            # don't publish or log when there's no movement
            pass

        self.current_key = 0 # reset key after processing

def get_movement(pressed_key, maxPositionDelta) -> list:
    '''
    returns how far the pointer will move on the screen based on key inputs
    '''
    
    key_to_dir = {
        68: (1, 0, 0),   # D
        65: (-1, 0, 0),  # A
        81: (0, -1, 0),  # Q
        69: (0, 1, 0),   # E
        87: (0, 0, 1),   # W
        83: (0, 0, -1),  # S
    }
    dir_vec = key_to_dir.get(pressed_key, (0, 0, 0)) # default to no movement
    movementDeltaVec = [d * maxPositionDelta for d in dir_vec]
    return movementDeltaVec

def main(args=None):
    rclpy.init(args=args)
    teleop_input_node = teleopInputNode()
    rclpy.spin(teleop_input_node)
    # The cleanup function will be called automatically on shutdown (e.g., Ctrl+C)
    teleop_input_node.destroy_node()
    rclpy.shutdown()