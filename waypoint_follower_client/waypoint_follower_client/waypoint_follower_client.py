import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from stero_nav_msgs.action import SteroNavWaypointFollow
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
import threading
import numpy as np

custom_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class WaypointFollowerClient(Node):
    def __init__(self):
        super().__init__("WaypointFollowerClient")
        self._waypoint_client = ActionClient(self, SteroNavWaypointFollow, "/stero_nav_waypoint",
                                             goal_service_qos_profile=custom_qos, result_service_qos_profile=custom_qos)
        self._finished = False
        # self._points = [(-4.0, 1.7), (-0.75, 2.75), (0.0, 0.0), (0.75, -2.4), (4.0, -1.7), (0.0, -5.2)]
        #
        # self._points = [(0.0, 0.0), (0.75, -2.4), (4.0, -1.7)]

        self._points = [(4.0, -1.7), (0.75, -2.4), (0.0, 0.0),
                        (-4.0, 1.7)]

        # self._points = [(-4.0, 1.7), (0.0, 0.0), (4.0, -1.7)]
        self._percentage_completed = []

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self._finished = True
            return
        self.get_logger().info('Goal accepted :)')

        self._done_future = goal_handle.get_result_async()
        self._done_future.add_done_callback(self._done_callback)

    def _done_callback(self, future):
        self._end_result = future.result()
        self.get_logger().info("Desired pose achieved!")
        np.savetxt("/home/vistek528/Desktop/percentage_completed.csv", np.array(self._percentage_completed))
        self._finished = True

    def run(self):
        self.get_logger().info("Starting move...")

        action_request = SteroNavWaypointFollow.Goal()
        action_request.waypoints = self._points_to_point_msg_list(self._points)
        action_request.header.frame_id = "map"
        action_request.header.stamp = self.get_clock().now().to_msg()

        self._waypoint_client.wait_for_server(10)
        future = self._waypoint_client.send_goal_async(action_request, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._goal_response_callback)

        while not self._finished:
            pass

        self.get_logger().info("Move finished successfully!")

    def _feedback_callback(self, msg: SteroNavWaypointFollow.Feedback):
        self._percentage_completed.append(msg.feedback.percentage_completed)
        self.get_logger().info(f"Percentage completed: {msg.feedback.percentage_completed:.2f}%")

    @staticmethod
    def _points_to_point_msg_list(points: list) -> list[Point]:
        pose_msgs = []
        for point in points:
            msg = Point()
            msg.x = point[0]
            msg.y = point[1]
            pose_msgs.append(msg)
        return pose_msgs



def main():
    rclpy.init()

    node = WaypointFollowerClient()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.run()
    node.destroy_node()
    executor.remove_node(node)
    executor.shutdown()
    spin_thread.join()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
