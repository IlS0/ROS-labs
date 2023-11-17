import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_turtle_interface.action import MessageTurtleCommands


class TurtleActionClient(Node):

    def __init__(self):
        super().__init__('action_turtle_client')
        self._action_client = ActionClient(self, MessageTurtleCommands, 'MessageTurtleCommands')

        self.goal_future = None
        self.result_future = None


    def send_goal(self, order):
        goal_msg = MessageTurtleCommands.Goal()

        for cmd,val in order:
            goal_msg.command  = cmd

            if goal_msg.command == 'forward':
                goal_msg.s = val
                goal_msg.angle = 0
            else:
                goal_msg.s = 0
                goal_msg.angle = val

            self._action_client.wait_for_server()
            self.goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.get_feedback_callback)
            #self.get_logger().info(f"goal_future type: {type(self.goal_future)}")
            self.goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.result}")



    def get_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance: {feedback.odom:4.f}')


def main(args=None):
    rclpy.init(args=args)

    action_turtle_client = TurtleActionClient()

    orders = [['forward', 2],['turn_right', 90],['forward', 1]]
    
    action_turtle_client.send_goal(orders)

    rclpy.spin(action_turtle_client)

    action_turtle_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()