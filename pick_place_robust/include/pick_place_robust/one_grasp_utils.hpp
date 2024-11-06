#ifndef ROS2_MASTER_ONE_GRASP_UTILS_HPP
#define ROS2_MASTER_ONE_GRASP_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/gazebo_msgs/srv/get_entity_state.hpp>
#include <Eigen/Eigen>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

void shutdownExecutor(rclcpp::executors::SingleThreadedExecutor &executor, std::thread &spinner);
std::optional<Eigen::Isometry3d> getItemPosition(const rclcpp::Logger& logger, rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr& client, const std::string& link_name, const std::string& reference_link);
void publishGrasp(moveit_visual_tools::MoveItVisualTools& visual_tools, const moveit::core::JointModelGroup* jmg, const Eigen::Isometry3d& T_B_O, const Eigen::Isometry3d& T_B_arm_tool_link_pre, const Eigen::Isometry3d& T_B_arm_tool_link_gr);


#endif //ROS2_MASTER_ONE_GRASP_UTILS_HPP
