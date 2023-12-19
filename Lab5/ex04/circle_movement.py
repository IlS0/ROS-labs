import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from math import pi

class Circle(Node):

    def __init__(self):
        super().__init__('circle')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_to_goal)

    def move_to_goal(self):
        tw = Twist()
        tw.linear.x = 1.0
        tw.angular.z = pi / 2 
        self.publisher.publish(tw)

def main():
    rclpy.init()
    circle= Circle()
    rclpy.spin(circle)
    circle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()