import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/robot/scan', self.laser_callback, 1)
        self.timer = self.create_timer(0.2, self.go_forward)
        self.msg = LaserScan()

    def laser_callback(self, data):
        self.msg = data

    def go_forward(self):
        message = Twist()
        isObstacle = False
        range = 25        
        sight_field = self.msg.ranges[359-range:359] + self.msg.ranges[:range]

        close_points = [point for point in sight_field if point < 0.5]

        if len(close_points) >= 10: #большое препятствие
            self.get_logger().info('An obstacle was founded')
            isObstacle = True

        message.linear.x = 0.0 if isObstacle else 0.2
        self.publisher.publish(message)            

def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()