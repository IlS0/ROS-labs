import rclpy
from rclpy.node import Node
from math import pi
from geometry_msgs.msg import Twist


class Circle(Node):
    def __init__(self):
        super().__init__('circle')
        self.i = 0
        self.dir = 1
        self.max = 60

        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_to_goal)
        self.radius = self.declare_parameter('radius', 0.5).get_parameter_value().double_value

    def move_to_goal(self):
        self.i = self.i + 1
        if self.i > self.max:
            self.max = self.max + 60
            self.dir = self.dir * -1
            
        radius = self.radius
        tw = Twist()
        tw.linear.x = radius * 2 * pi
        tw.angular.z = self.dir * radius * 2 * pi
        self.publisher.publish(tw)

def main():
    rclpy.init()
    circle= Circle()
    rclpy.spin(circle)
    circle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()