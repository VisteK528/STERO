import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav2_msgs.action import NavigateToPose
import threading

custom_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

class NavTester(Node):
    def __init__(self):
        super().__init__("NavTester")
        self.declare_parameter("room_name", value="living_room")
        self.declare_parameter("difficulty", value="easy")

        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose",
                                           goal_service_qos_profile=custom_qos,
                                           result_service_qos_profile=custom_qos)
        self._destination_room = self.get_parameter("room_name").get_parameter_value().string_value

        self._center_positions = {
            "corridor1": {"easy": (-0.75, 2.75), "normal": (-0.75, 2.75), "hard": (-0.75, 2.75)},  # done
            "corridor2": {"easy": (0.75, -2.4), "normal": (0.0, 0.0), "hard": (0.0, 0.0)},  # done
            "corridor": {"easy": (0.0, 0.0), "normal": (0.0, 0.0), "hard": (0.0, 0.0)}, #done
            "living_room": {"easy": (4.0, 1.7), "normal": (5.5, 2.0), "hard": (2.4, 3.3)}, #done
            "kitchen": {"easy": (4.0, -1.7), "normal": (4.55, -2.54), "hard": (2.55, -2.92)}, #done
            "room1": {"easy": (-4.0, 1.7), "normal": (-4.0, 3.0), "hard": (-2.9, 3.0)}, #done
            "room2": {"easy": (-4.0, -1.7), "normal": (-4.69, -2.4), "hard": (-2.69, -0.56)}, #done
            "wc": {"easy": (0.0, -5.2), "normal": (-1.24, -5.7), "hard": (1.05, -4.36)}, #TODO
        }

        self._finished = False
        self._end_result = None

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
        self._finished = True

    def run(self):
        self.get_logger().info("Starting move...")

        selected_room = self.get_parameter("room_name").get_parameter_value().string_value
        difficulty = self.get_parameter("difficulty").get_parameter_value().string_value
        if selected_room:
            self.get_logger().info(f"Selected move to room: {selected_room}")

            selected_room_dict = self._center_positions.get(selected_room, (0, 0))
            selected_room_coordinates = selected_room_dict.get(difficulty)
            self.get_logger().info(f"Selected difficulty: {difficulty}")
            self.get_logger().info(f"Destination coordinates (X,Y) = ({selected_room_coordinates[0]:.2f},{selected_room_coordinates[1]:.2f})")

            action_request = NavigateToPose.Goal()
            action_request.pose.pose.position.x = selected_room_coordinates[0]
            action_request.pose.pose.position.y = selected_room_coordinates[1]
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

            if self._end_result is not None:
                if self._end_result.result.error_code != 0:
                    self.get_logger().info("Move failed... Exiting...")
                else:
                    self.get_logger().info("Move finished successfully!")
            else:
                self.get_logger().info("Move failed... Exiting...")

        else:
            self.get_logger().info("No destination room has been selected! Exiting...")




def main():
    rclpy.init()

    node = NavTester()

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
