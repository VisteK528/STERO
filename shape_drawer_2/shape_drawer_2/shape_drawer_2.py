import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from scipy.spatial.transform import Rotation as R
import threading
import time
import math


def quaternion_to_yaw(qx, qy, qz, qw):
    # Create a rotation object from the quaternion
    rotation = R.from_quat([qx, qy, qz, qw])

    # Convert to Euler angles (roll, pitch, yaw)
    roll, pitch, yaw = rotation.as_euler('xyz', degrees=True)

    # Normalize yaw to 0-360 range
    yaw_normalized = yaw % 360
    return yaw_normalized


class ShapeDrawer(Node):
    def __init__(self):
        super().__init__("ShapeDrawer")

        self._odom_subscriber = self.create_subscription(Odometry, "/mobile_base_controller/odom", self._update_odom, 10, callback_group=ReentrantCallbackGroup())
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 10, callback_group=ReentrantCallbackGroup())

        self._x = 0
        self._y = 0
        self._angle = 0
        self._tolerance = 0.1

    def _update_odom(self, msg: Odometry):

        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        self._angle = quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)

    def _rotate_angle(self, deg):
        msg = Twist()

        while True:
            error = deg - self._angle
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360

            if abs(error) < self._tolerance:
                self._publisher.publish(Twist())
                break

            angular_speed = 0.3
            msg.angular.z = angular_speed if error > 0 else -angular_speed
            self._publisher.publish(msg)
            time.sleep(0.01)

    def _drive_length(self, meters):
        start_x = self._x
        start_y = self._y

        msg = Twist()
        msg.linear.x = 0.2

        while math.sqrt(pow(start_x - self._x, 2) + pow(start_y - self._y, 2)) < meters:
            self._publisher.publish(msg)
            time.sleep(0.01)

        self._publisher.publish(Twist())

    def run(self):
        self._rotate_angle(180)

        for i in range(1, 7):
            self._drive_length(1)

            rotation_angle = 180 - 60 * i
            if rotation_angle < 0:
                rotation_angle += 360
            self._rotate_angle(rotation_angle)

        self.get_logger().info("Finished")


def main():
    rclpy.init()

    node = ShapeDrawer()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(2)
    node.run()
    node.destroy_node()
    executor.remove_node(node)
    executor.shutdown()
    spin_thread.join()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
