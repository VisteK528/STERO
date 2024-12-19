import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose

import threading


class NavTester(Node):
    def __init__(self):
        super().__init__("NavTester")
        self.declare_parameter("room_name", value="living_room")

        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self._destination_room = self.get_parameter("room_name").get_parameter_value().string_value

        # self._pose_x = 3.5
        # self._pose_y = -1.5
        # self._pose_x = - 4.8
        # self._pose_y = 2.4

        self._pose_x = 0.0
        self._pose_y = -5.8

        self._finished = False

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self._finished = False
            return
        self.get_logger().info('Goal accepted :)')

        self._done_future = goal_handle.get_result_async()
        self._done_future.add_done_callback(self._done_callback)

    def _done_callback(self, future):
        self.get_logger().info("Desired pose achieved!")
        self._finished = True

    def run(self):
        self.get_logger().info("Starting move...")
        action_request = NavigateToPose.Goal()
        action_request.pose.pose.position.x = self._pose_x
        action_request.pose.pose.position.y = self._pose_y
        action_request.pose.pose.position.z = 0.0

        action_request.pose.pose.orientation.x = 0.0
        action_request.pose.pose.orientation.y = 0.0
        action_request.pose.pose.orientation.z = 0.0
        action_request.pose.pose.orientation.w = 1.0

        action_request.pose.header.frame_id = "map"

        self._action_client.wait_for_server(1)
        future = self._action_client.send_goal_async(action_request)
        future.add_done_callback(self._goal_response_callback)

        while not self._finished:
            pass

        self.get_logger().info("Move finished!")




def main():
    rclpy.init()

    node = NavTester()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.run()
    time.sleep(1)
    node.destroy_node()
    executor.remove_node(node)
    executor.shutdown()
    spin_thread.join()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
