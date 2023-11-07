import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_turtle_interface.action import MessageTurtleCommands
from geometry_msgs.msg import Twist


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

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        command = goal_handle.request.command
        s = goal_handle.request.s
        angle = goal_handle.request.angle

        feedback_msg = MessageTurtleCommands.Feedback()
        tw = Twist()	
        
        if command == 'forward':
            self.get_logger().info(f'Goal: {command}, {s} metres.')
            
            for _ in range(s):
                tw.linear.x += 1.0
                self.publisher.publish(tw)

                self.odom += 1
                feedback_msg.odom = self.odom

                self.get_logger().info(f'Feedback: odom {feedback_msg.odom} metres')
                goal_handle.publish_feedback(feedback_msg)

                time.sleep(1)
                
        elif command == 'turn_left':
            self.get_logger().info(f'Goal: {command}, {angle} degrees.')

            tw.angular.z = angle * np.pi / 180
            self.publisher.publish(tw)

            self.get_logger().info(f'Feedback: odom {feedback_msg.odom} metres')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        elif command == 'turn_right':
            self.get_logger().info(f'Goal: {command}, {angle} degrees.')

            tw.angular.z = -1 * angle * np.pi / 180
            self.publisher.publish(tw)

            self.get_logger().info(f'Feedback: odom {feedback_msg.odom} metres')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        else:
            self.get_logger().error(f'Invalid command received: {command}')
            raise ValueError('Invalid command :(')
        
        self.get_logger().info('Goal reached :)')


        goal_handle.succeed()

        result = MessageTurtleCommands.Result()
        result.result = True
        return result


def main(args=None):
    rclpy.init(args=args)

    action_turtle_server = TurtleActionServer()

    rclpy.spin(action_turtle_server)

    action_turtle_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()