import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from stero_nav_msgs.action import SteroNavWaypointFollow
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Pose, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Path
import math
from enum import Enum

custom_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

class RotationDirection(Enum):
    NOROTATION = 0
    CLOCKWISE = 1
    ANTICLOCKWISE = 2


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

        self._velocity_subscriber= self.create_subscription(Twist, "/mobile_base_controller/cmd_vel_unstamped",
                                                            self._mobile_base_velocity_callback,
                                                            10,
                                                            callback_group=ReentrantCallbackGroup())

        self._head_controller_publisher = self.create_publisher(JointTrajectory, "/head_controller/joint_trajectory", qos_profile=custom_qos, callback_group=ReentrantCallbackGroup())
        self._head_controller_timer = self.create_timer(1, self._head_control_callback, callback_group=ReentrantCallbackGroup())

        self._current_pose = None
        self._current_rotation = RotationDirection.NOROTATION

        self._waypoints = None


        # New variant
        self._paths = []
        self._paths_lengths = []
        self._current_waypoint = None

        self._already_driven_paths = None
        self._already_driven_paths_lengths = None

        self._current_path = None
        self._current_path_length = 0.0
        self._full_path_length = 0.0


    def setup(self):
        self.get_logger().info("Starting Waypoint follower server...")
        while self._current_pose is None:
            rclpy.spin_once(self)

        self.get_logger().info(f"Starting work at P=({self._current_pose.pose.position.x:.2f},{self._current_pose.pose.position.y:.2f})")
        self._nav.setInitialPose(self._current_pose)
        self.get_logger().info("Waypoint follower server has started!")

    def reset_before_action(self):
        self._waypoints = None

        self._paths = []
        self._paths_lengths = []
        self._current_waypoint = None

        self._already_driven_paths = None
        self._already_driven_paths_lengths = None

        self._current_path = None
        self._current_path_length = 0.0
        self._full_path_length = 0.0

    def _mobile_base_velocity_callback(self, msg: Twist):
        if msg.angular.z >= 0.2:
            self._current_rotation = RotationDirection.ANTICLOCKWISE
        elif msg.angular.z <= -0.2:
            self._current_rotation = RotationDirection.CLOCKWISE
        else:
            self._current_rotation = RotationDirection.NOROTATION

    def _head_control_callback(self):
        msg = JointTrajectory()
        msg.joint_names = ["head_1_joint", "head_2_joint"]

        point = JointTrajectoryPoint()
        point.time_from_start.sec = 1
        point.velocities = [0.0, 0.0]
        point.accelerations = [0.0, 0.0]

        if self._current_rotation == RotationDirection.CLOCKWISE:
            #self.get_logger().info("Rotating clockwise!")
            point.positions = [-math.pi/4, 0.0]
        elif self._current_rotation == RotationDirection.ANTICLOCKWISE:
            #self.get_logger().info("Rotating anticlockwise!")
            point.positions = [math.pi/4, 0.0]
        else:
            point.positions = [0.0, 0.0]

        msg.points.append(point)
        self._head_controller_publisher.publish(msg)


    def _update_path_callback(self, msg: Path):
        self._current_path = msg.poses
        self._current_path_length = self.calculate_path_length(self._current_path)


    def _update_pose_callback(self, msg: PoseWithCovarianceStamped):
        # self.get_logger().info("Updated")
        self._current_pose = PoseStamped()
        self._current_pose.pose = msg.pose.pose
        self._current_pose.header = msg.header

        if self._current_waypoint is not None:
            if self._current_waypoint > 0:
                self._already_driven_paths[self._current_waypoint-1].append(self._current_pose)


    def _drive_waypoints_callback(self, goal_request):
        self.reset_before_action()
        self._waypoints = goal_request.waypoints
        return GoalResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        feedback_message = SteroNavWaypointFollow.Feedback()

        waypoints_converted = self._point_msgs_to_pose_msgs(self._waypoints)
        waypoints_converted.insert(0, self._current_pose)

        self._full_path_length = 0.0
        self._already_driven_paths = [[] for _ in range(len(self._waypoints) - 1)]
        self._already_driven_paths_lengths = [0 for _ in range(len(self._waypoints) - 1)]
        for i in range(1, len(waypoints_converted)):
            path = self._nav.getPath(waypoints_converted[i-1], waypoints_converted[i], use_start=True)
            path_length = self.calculate_path_length(path.poses)
            self._paths.append(path)

            self._paths_lengths.append(path_length)
            self._full_path_length += path_length

        self._nav.waitUntilNav2Active()
        self._nav.followWaypoints(self._point_msgs_to_pose_msgs(self._waypoints))

        rate = self.create_rate(5)
        while self._current_path is None:
            pass

        self._start_path_length = self._current_path_length
        while not self._nav.isTaskComplete():
            current_waypoint = self._nav.getFeedback().current_waypoint
            self._current_waypoint = current_waypoint
            percentage_completed = self.calculate_current_progress(current_waypoint)


            feedback_message.percentage_completed = float(percentage_completed)
            self.get_logger().info(f"Percentage completed: {percentage_completed:.2f}%")

            goal_handle.publish_feedback(feedback_message)
            rate.sleep()

        self._current_path = []
        percentage_completed = self.calculate_current_progress(self._current_waypoint)
        feedback_message.percentage_completed = float(percentage_completed)
        self.get_logger().info(f"Percentage completed: {percentage_completed:.2f}%")
        goal_handle.publish_feedback(feedback_message)
        rate.sleep()

        goal_handle.succeed()
        result = SteroNavWaypointFollow.Result()
        result.status = 0

        self.get_logger().info("Job finished")
        return result

    def calculate_current_progress(self, current_waypoint: int) -> float:
        current_path_to_be_driven = self.calculate_path_length(self._current_path)
        current_path_already_driven = self.calculate_path_length(self._already_driven_paths[self._current_waypoint - 1])
        already_driven = sum(self._already_driven_paths_lengths[:(self._current_waypoint - 1)])
        yet_to_be_driven = sum(self._paths_lengths[self._current_waypoint+1:])

        self._already_driven_paths_lengths[self._current_waypoint - 1] = current_path_already_driven
        self.get_logger()
        driven = already_driven + current_path_already_driven
        full_path_length = already_driven + current_path_already_driven + current_path_to_be_driven + yet_to_be_driven
        self.get_logger().info(f"Driven: {driven:.3f} /  {full_path_length:.3f} / {self._current_waypoint} / {current_path_to_be_driven:.3f} / {yet_to_be_driven:.3f}")
        return (driven / full_path_length) * 100.0


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
