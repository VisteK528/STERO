#ifndef ROS2_MASTER_ONE_GRASP_UTILS_HPP
#define ROS2_MASTER_ONE_GRASP_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/gazebo_msgs/srv/get_entity_state.hpp>
#include <Eigen/Eigen>

void shutdownExecutor(rclcpp::executors::SingleThreadedExecutor &executor, std::thread &spinner);
std::optional<Eigen::Isometry3d> getItemPosition(const rclcpp::Logger& logger, rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr& client, const std::string& link_name, const std::string& reference_link);


#endif //ROS2_MASTER_ONE_GRASP_UTILS_HPP
