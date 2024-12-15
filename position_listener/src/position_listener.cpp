#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/nav_msgs/msg/odometry.hpp>
#include <string>



void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::string coordinates;
    coordinates += "X: " + std::to_string(msg->pose.pose.position.x) + "\t";
    coordinates += "Y: " + std::to_string(msg->pose.pose.position.y) + "\t";
    coordinates += "Z: " + std::to_string(msg->pose.pose.position.z);

    std::cout<<coordinates<<std::endl;
}


using std::placeholders::_1;

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
    "position_listener",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("position_listener");


    RCLCPP_INFO(logger, "Starting the position_listener node");

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "/mobile_base_controller/odom",
            10,
            std::bind(&odometryCallback, std::placeholders::_1));


    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}