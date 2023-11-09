import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from concurrent.futures import Future

import sys
from math import pi, sqrt, atan2



class MoveToGoal(Node):
    def __init__(self, x, y, angle):
        super().__init__("move_to_goal_node")

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.set_pose_callback, 10)
        self.current_pose = Pose()
        self.goal_pose = Pose(x = float(x), y = float(y), theta = float(angle) * pi / 180)
        self.future = Future()

        #таймер по заветам лектора
        self.timer = self.create_timer(1, self.move_to_goal)
        #константа точности
        self.delta = 0.05
        self.odom = 0

    def set_pose_callback(self, pose):
        self.current_pose = pose


    def send_turtle_msg(self, s, angle):
        tw = Twist()
        tw.linear.x = float(s)
        tw.angular.z = float(angle)

        self.publisher.publish(tw)


    def move_to_goal(self):
        x_d = self.goal_pose.x - self.current_pose.x
        y_d = self.goal_pose.y - self.current_pose.y

        alpha = atan2(y_d, x_d) # угол в радианах
        angle_target = alpha - self.current_pose.theta

        s_target = sqrt(x_d**2 + y_d**2)
        s_target *= 0.3
        self.odom+=s_target
        
        self.get_logger().info(f"Current position: {self.current_pose.x:.4f} {self.current_pose.y:.4f} {self.current_pose.theta:.4f}")
        self.get_logger().info(f"Goal position: {self.goal_pose.x:.4f} {self.goal_pose.y:.4f} {self.goal_pose.theta:.4f}")
        self.get_logger().info(f"Odom: {self.odom:.4f}")

        if abs(s_target) > self.delta:
            self.send_turtle_msg(s_target, angle_target)
            return

        self.timer.destroy()

        angle_target = self.goal_pose.theta - self.current_pose.theta
        
        self.send_turtle_msg(0.0, angle_target)
        self.get_logger().info(f"Goal Reached")
        self.future.set_result(None)

def main():
    rclpy.init()

    moving = MoveToGoal(sys.argv[1], sys.argv[2], sys.argv[3])

    #print(sys.argv[1], sys.argv[2], sys.argv[3])

    rclpy.spin_until_future_complete(moving, moving.future) 
    
    moving.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()