import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_turtle_interface.action import MessageTurtleCommands
from geometry_msgs.msg import Twist

from turtlesim.msg import Pose
from rclpy.executors import MultiThreadedExecutor

from math import sqrt
import numpy as np

class TurtleActionServer(Node):

    def __init__(self):
        super().__init__('action_turtle_server')

        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'MessageTurtleCommands',
            self.execute_callback)
        
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.odom = 0

        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.set_pose_callback, 10)
        self.current_pose = Pose()
        self.last_pose = Pose()


    def set_pose_callback(self, pose):
        self.current_pose = pose

    def odom_publish_feedback(self):
        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = self.calc_distance(self.last_pose, self.current_pose)
        self.goal_handle.publish_feedback(feedback_msg)


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        command = goal_handle.request.command
        s = goal_handle.request.s
        angle = goal_handle.request.angle

        feedback_msg = MessageTurtleCommands.Feedback()
        tw = Twist()	
        
        if command == 'forward':
            self.get_logger().info(f'Goal: {command}, {s} metres.')
            tw.linear.x = s
            self.publisher.publish(tw)
                
        elif command == 'turn_left':
            self.get_logger().info(f'Goal: {command}, {angle} degrees.')

            tw.angular.z = angle * np.pi / 180
            self.publisher.publish(tw)

            #self.get_logger().info(f'Feedback: odom {feedback_msg.odom} metres')
            #goal_handle.publish_feedback(feedback_msg)
            #time.sleep(1)

        elif command == 'turn_right':
            self.get_logger().info(f'Goal: {command}, {angle} degrees.')

            tw.angular.z = -1 * angle * np.pi / 180
            self.publisher.publish(tw)

            #self.get_logger().info(f'Feedback: odom {feedback_msg.odom} metres')
            #goal_handle.publish_feedback(feedback_msg)
            #time.sleep(1)

        else:
            self.get_logger().error(f'Invalid command received: {command}')
            raise ValueError('Invalid command :(')
        
        while not self.current_pose.linear_velocity or self.current_pose.angular_velocity:
            pass
        
        self.last_pose = self.current_pose
        self.goal_handle = goal_handle
        feedback_timer = self.create_timer(0.5, self.odom_publish_feedback)

        while self.current_pose.linear_velocity or self.current_pose.angular_velocity:
            pass

        feedback_timer.cancel()
        self.get_logger().info('Goal reached :)')


        goal_handle.succeed()

        result = MessageTurtleCommands.Result()
        result.result = True
        return result
    

    def calc_distance(fst_pose, snd_pose):
        return sqrt((fst_pose.x - snd_pose.x)**2 + (fst_pose.y - snd_pose.y)**2) 


def main(args=None):
    rclpy.init(args=args)

    action_turtle_server = TurtleActionServer()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(action_turtle_server)
    executor.spin()

    #rclpy.spin(action_turtle_server)

    action_turtle_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()