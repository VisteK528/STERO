import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from stero_nav_msgs.action import SteroNavWaypointFollow
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Path
import math

custom_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class WaypointFollowerServer(Node):
    def __init__(self):
        super().__init__("WaypointFollowerServer")
        self._nav = BasicNavigator()
        self._nav_executor = SingleThreadedExecutor()
        self._nav.executor = self._nav_executor

        self._waypoint_server = ActionServer(
            self,
            SteroNavWaypointFollow,
            "/stero_nav_waypoint",
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self._drive_waypoints_callback,
            execute_callback=self._execute_callback,
        )

        self._amcl_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._update_pose_callback, qos_profile=custom_qos, callback_group=ReentrantCallbackGroup()
        )

        self._path_subscriber = self.create_subscription(
            Path, "/plan", self._update_path_callback, 100, callback_group=ReentrantCallbackGroup()
        )

        self._current_pose = None

        self._current_path_length = 0.0
        self._initial_pose = [(0.0, 0.0)]
        self._waypoints = None


        # New variant
        self._paths = []
        self._paths_lengths = []
        self._current_waypoint = None

        self._current_path = None
        self._current_path_length = 0.0
        self._full_path_length = 0.0
        self._requested_computation = False


    def setup(self):
        self._nav.setInitialPose(self._point_to_pose_msgs(self._initial_pose)[0])
        self.get_logger().info("Waypoint follower server has started!")

    def _update_path_callback(self, msg: Path):
        self._current_path = msg.poses
        self._current_path_length = self.calculate_path_length(self._current_path)
        self.get_logger().info("Path updated")

        # if self._current_waypoint is not None and not self._requested_computation:
        #     if self._current_waypoint > 0:
        #         start = (self._waypoints[self._current_waypoint-1].x,
        #                  self._waypoints[self._current_waypoint-1].y)
        #         goal = (self._waypoints[self._current_waypoint].x,
        #                  self._waypoints[self._current_waypoint].y)
        #         start, goal = self._point_to_pose_msgs([start, goal])
        #         path = self._nav.getPath(start, goal, use_start=True)
        #
        #         self._paths[self._current_waypoint - 1] = path
        #         self._paths_lengths[self._current_waypoint-1] = self.calculate_path_length(path.poses)
        #         self._full_path_length = sum(self._paths_lengths)
        #         self._requested_computation = True
        # else:
        #     self._requested_computation = False

    def _update_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.get_logger().info("Updated")
        self._current_pose = PoseStamped()
        self._current_pose.pose = msg.pose.pose
        self._current_pose.header = msg.header

    def _drive_waypoints_callback(self, goal_request):
        self._waypoints = goal_request.waypoints
        return GoalResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        feedback_message = SteroNavWaypointFollow.Feedback()

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self._nav.get_clock().now().to_msg()
        initial_pose.pose.position.x = self._current_pose.pose.position.x
        initial_pose.pose.position.y = self._current_pose.pose.position.y
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self._nav.setInitialPose(initial_pose)

        waypoints_converted = self._point_msgs_to_pose_msgs(self._waypoints)
        waypoints_converted.insert(0, initial_pose)

        self._full_path_length = 0.0
        for i in range(1, len(waypoints_converted)):
            path = self._nav.getPath(waypoints_converted[i-1], waypoints_converted[i], use_start=True)
            path_length = self.calculate_path_length(path.poses)
            self._paths.append(path)
            self._paths_lengths.append(path_length)
            self._full_path_length += path_length

        self._nav.waitUntilNav2Active()
        self._nav.followWaypoints(self._point_msgs_to_pose_msgs(self._waypoints))

        rate = self.create_rate(1)
        while self._current_path is None:
            pass

        self._start_path_length = self._current_path_length
        while not self._nav.isTaskComplete():
            current_waypoint = self._nav.getFeedback().current_waypoint

            # if self._current_waypoint is not None:
            #     if current_waypoint > self._current_waypoint and self._current_waypoint != 0:
            #         start = (self._waypoints[self._current_waypoint - 1].x,
            #                  self._waypoints[self._current_waypoint - 1].y)
            #         goal = (self._waypoints[self._current_waypoint].x,
            #                 self._waypoints[self._current_waypoint].y)
            #         start, goal = self._point_to_pose_msgs([start, goal])
            #         path = self._nav.getPath(start, goal, use_start=True)
            #
            #         self._paths[self._current_waypoint - 1] = path
            #         self._paths_lengths[
            #             self._current_waypoint - 1] = self.calculate_path_length(
            #             path.poses)
            #         self._full_path_length = sum(self._paths_lengths)

            self._current_waypoint = current_waypoint
            percentage_completed = self.calculate_current_progress(current_waypoint)
            self.get_logger().info(f"{current_waypoint}")

            feedback_message.percentage_completed = float(percentage_completed)
            self.get_logger().info(
                 f"Percentage completed: {percentage_completed:.2f}%")

            goal_handle.publish_feedback(feedback_message)
            rate.sleep()

        self.get_logger().info("Exiting...")
        goal_handle.succeed()

        result = SteroNavWaypointFollow.Result()
        result.status = 0
        self.get_logger().info("Job finished")
        return result

    def calculate_current_progress(self, current_waypoint: int) -> float:
        driven_distance = sum(self._paths_lengths[:current_waypoint+1]) - self.calculate_path_length(self._current_path)

        return (driven_distance/ self._full_path_length)* 100.0

    @staticmethod
    def calculate_distance_between_poses(pose1: Pose, pose2: Pose) -> float:
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        distance = math.sqrt(pow(dx, 2) + pow(dy, 2))
        return distance

    def calculate_path_length(self, path: list[PoseStamped]) -> float:
        length = 0
        for i in range(1, len(path)):
            length += self.calculate_distance_between_poses(path[i].pose, path[i-1].pose)
        return length

    def _point_to_pose_msgs(self, points: list) -> list[PoseStamped]:
        pose_msgs = []
        for point in points:
            msg = PoseStamped()
            msg.pose.position.x = point[0]
            msg.pose.position.y = point[1]
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            pose_msgs.append(msg)
        return pose_msgs

    @staticmethod
    def _point_msgs_to_pose_msgs(point_msgs_list: list[Point]) -> list[PoseStamped]:
        pose_msgs = []
        for point in point_msgs_list:
            msg = PoseStamped()
            msg.pose.position.x = point.x
            msg.pose.position.y = point.y
            msg.header.frame_id = "map"
            pose_msgs.append(msg)
        return pose_msgs


def main():
    rclpy.init()

    node = WaypointFollowerServer()
    node.setup()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
