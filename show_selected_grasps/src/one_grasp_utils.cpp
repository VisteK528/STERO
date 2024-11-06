#include "../include/show_selected_grasps/one_grasp_utils.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::literals::chrono_literals;


void shutdownExecutor(rclcpp::executors::SingleThreadedExecutor &executor, std::thread &spinner) {
    executor.cancel();
    if (spinner.joinable()) {
        spinner.join();
    }
    rclcpp::shutdown();
}

std::optional<Eigen::Isometry3d> getItemPosition(const rclcpp::Logger& logger, rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr& client, const std::string& link_name, const std::string& reference_link)
{
    std::string message = "Getting position of " + link_name + " in " + reference_link + " frame...";
    RCLCPP_INFO(logger, message.c_str());
    Eigen::Isometry3d frame;

    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = link_name;
    request->reference_frame = reference_link;

    while(!client->wait_for_service(std::chrono::seconds(1s))){
        if(!rclcpp::ok()){
            RCLCPP_ERROR(logger, "Interrupted. Exiting.");
            return std::nullopt;
        }
        RCLCPP_INFO(logger, "service not available, waiting...");
    }


    auto future_result1 = client->async_send_request(request);
    if(future_result1.wait_for(3s) == std::future_status::ready){
        auto resp = future_result1.get();
        tf2::fromMsg(resp->state.pose, frame);
        return frame;
    }

    RCLCPP_ERROR(logger, "Failed to call service /get_entity_state");
    return std::nullopt;
}

void publishGrasp(moveit_visual_tools::MoveItVisualTools& visual_tools, const moveit::core::JointModelGroup* jmg, const Eigen::Isometry3d& T_B_O, const Eigen::Isometry3d& T_B_arm_tool_link_pre, const Eigen::Isometry3d& T_B_arm_tool_link_gr){
    moveit_msgs::msg::Grasp vis_gr;
    vis_gr.id = "grasp_1";
    vis_gr.grasp_pose.header.frame_id = "arm_tool_link";
    vis_gr.grasp_pose.pose = tf2::toMsg(T_B_arm_tool_link_gr);

    // 2. Display arm_tool_link pose during grip phase
    visual_tools.publishAxis(T_B_arm_tool_link_gr);

    // 3. Display arm_tool_link pose during pre grip phase
    visual_tools.publishAxis(T_B_arm_tool_link_pre);

    // 4. Display cube pose
    visual_tools.publishAxis(T_B_O);
    visual_tools.trigger();

    std::vector<moveit_msgs::msg::Grasp> grasps;
    grasps.push_back(vis_gr);
    visual_tools.publishGrasps({grasps}, jmg);
}