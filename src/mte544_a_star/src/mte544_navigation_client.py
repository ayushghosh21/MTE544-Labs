import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


from mte544_action_interfaces.action import Move2Goal


class AStarClient(Node):

    def __init__(self):
        super().__init__('a_star_action_client')
        self._action_client = ActionClient(self, Move2Goal, 'mte_544_a_star')

    def send_goal(self, initial_x, initial_y, goal_x, goal_y):
        goal_msg = Move2Goal.Goal()
        goal_msg.initial_x = initial_x
        goal_msg.initial_y = initial_y
        goal_msg.goal_x = goal_x
        goal_msg.goal_y = goal_y

        
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.reached_goal))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = AStarClient()

    initial_x = 0
    initial_y = 0
    goal_x = 10 
    goal_y = 10
    action_client.send_goal(initial_x, initial_y, goal_x, goal_y)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
