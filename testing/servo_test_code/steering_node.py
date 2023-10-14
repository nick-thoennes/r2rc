from adafruit_servokit import ServoKit
import rclpy
from rclpy.node import Node
from geometry_msgs import Twist
import geometry_msgs

class SteeringNode(Node):
    def __init__(self):
        super().__init__('steering_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.send_to_servo,
            10
        )
        self.subscription

    def send_to_servo(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    steering_node = SteeringNode()
    rclpy.spin(steering_node)
    steering_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
