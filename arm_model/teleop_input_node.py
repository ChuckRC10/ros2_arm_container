import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
            movementDeltaVector = get_movement(self.current_key, self.maxPositionDelta)
            twist_msg = Twist()
            twist_msg.linear.x = movementDeltaVector[0]
            twist_msg.linear.y = movementDeltaVector[1]
            twist_msg.linear.z = movementDeltaVector[2]
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0
            self.publisher_.publish(twist_msg)
            self.get_logger().info('Publishing movement delta: x: %.4f, y: %.4f, z: %.4f' % (movementDeltaVector[0], movementDeltaVector[1], movementDeltaVector[2]))

            self.current_key = 0 # reset key after processing

def get_movement(pressed_key, maxPositionDelta) -> list:
    '''
    returns how far the pointer will move on the screen based on key inputs
    '''
    xMovement = 0
    yMovement = 0
    zMovement = 0
    # get user input
    if pressed_key == 68:
        xMovement = maxPositionDelta
    if pressed_key == 65:
        xMovement = -maxPositionDelta
    if pressed_key == 81:
        yMovement = -maxPositionDelta
    if pressed_key == 69:
        yMovement = maxPositionDelta
    if pressed_key == 87:
        zMovement = maxPositionDelta
    if pressed_key == 83:
        zMovement = -maxPositionDelta

    movementDeltaVector = [xMovement, yMovement, zMovement]
    return movementDeltaVector

def main(args=None):
    rclpy.init(args=args)
    teleop_input_node = teleopInputNode()
    rclpy.spin(teleop_input_node)
    # The cleanup function will be called automatically on shutdown (e.g., Ctrl+C)
    teleop_input_node.destroy_node()
    rclpy.shutdown()