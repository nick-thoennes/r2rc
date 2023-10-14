import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

import numpy as np

#x_msg = Float64()
#z_msg = Float64()


#collect x and z data from /cmd_vel topic
#x is the left stick, used for forward and backward linear velocity, -2000 to 2000
#z is the right stick, used for right and left steering angle, -90 to 90
class Teleoperation_Node(Node):
    def __init__(self):
        super().__init__('Teleoperation_Node')
        self.subscriber = self.create_subscription(
                                Twist, 
                                'cmd_vel', 
                                self.send, 
                                10)
        self.vesc_publisher = self.create_publisher(
                                Float64, 
                                'commands/motor/speed',
                                10)
        self.servo_publisher = self.create_publisher(
                                Float64, 
                                'servo-steering', 
                                10)
        
    def send(self, msg):
        x_msg = Float64()
        x_msg.data = msg.linear.x
        self.vesc_publisher.publish(x_msg)

        z_msg = Float64()
        z_msg.data = msg.angular.z
        self.servo_publisher.publish(z_msg)


def main(args=None):
    print('Hi from teleoperation_node.')
    rclpy.init(args=args)
    teleoperation_node_ = Teleoperation_Node()
    rclpy.spin(teleoperation_node_)


if __name__ == '__main__':
    main()
