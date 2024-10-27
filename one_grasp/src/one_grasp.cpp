#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gazebo_msgs/gazebo_msgs/srv/get_entity_state.hpp>
#include <geometry_msgs/geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
//#include <moveit_msgs/moveit_msgs/msg/grasp.hpp>
#include <thread>
#include <Eigen/Eigen>


using namespace std::literals::chrono_literals;

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
    "one_grasp",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );


    // Create a ROS logger
    auto const logger = rclcpp::get_logger("one_grasp");

    RCLCPP_INFO(logger, "Starting node...");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm_torso");
    auto move_group_interface_gripper = MoveGroupInterface(node, "gripper");

    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
            node, "base_footprint", rviz_visual_tools::RVIZ_MARKER_TOPIC,
            move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    Eigen::Isometry3d T_B_O;
    Eigen::Isometry3d T_arm7_link_pre_gr_O = Eigen::Translation3d(0.0028227705405636633, 0.008221876185005018, 0.4088518719276147) * Eigen::Quaternion(0.02604331807245956, -0.017432560531374066, -0.707375654300129, -0.7061427158305615).normalized();
    Eigen::Isometry3d T_arm7_link_gr_O = Eigen::Translation3d(0.0023609, 0.0070378, 0.2781856) * Eigen::Quaternion(0.010816667244229508, -0.001105793243408453, -0.7079713127308999, -0.7061574875912008).normalized();
    Eigen::Isometry3d T_arm7_link_arm_tool_link = Eigen::Translation3d(0, 0, 0.046) * Eigen::Quaterniond(-0.5, 0.5, 0.5, 0.5).normalized();

    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client = node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();

    RCLCPP_INFO(logger, "Getting position of green_cube_3 in base_footprint frame...");
    request->name = "green_cube_3::link";
    request->reference_frame = "base_footprint";

    while(!client->wait_for_service(std::chrono::seconds(1s))){
        if(!rclcpp::ok()){
            RCLCPP_ERROR(logger, "Interrupted. Exiting.");
            return 0;
        }
        RCLCPP_INFO(logger, "service not available, waiting...");
    }

    auto future_result = client->async_send_request(request);
    if(future_result.wait_for(3s) == std::future_status::ready){
        auto resp = future_result.get();
        tf2::fromMsg(resp->state.pose, T_B_O);
    }else{
        RCLCPP_ERROR(logger, "Failed to call service /get_entity_state");

        // Shutdown ROS
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
    }

    RCLCPP_INFO(logger, "Computing position of arm_tool_link in base_footprint frame");
    Eigen::Isometry3d T_B_arm_tool_link_gr = T_B_O * T_arm7_link_gr_O.inverse() * T_arm7_link_arm_tool_link;
    Eigen::Isometry3d T_B_arm_tool_link_pre = T_B_O * T_arm7_link_pre_gr_O.inverse() * T_arm7_link_arm_tool_link;

    RCLCPP_INFO(logger, "Publishing frames...");
    moveit_visual_tools.publishAxis(T_B_O);
    moveit_visual_tools.publishAxis(T_B_arm_tool_link_gr);
    moveit_visual_tools.publishAxis(T_B_arm_tool_link_pre);
    moveit_visual_tools.trigger();


    // Gripping section (planning)
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // Open gripper
    move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.044, 0.044}));
    if(static_cast<bool>(move_group_interface_gripper.plan(plan))){
        move_group_interface_gripper.execute(plan);
    }
    else{
        // Shutdown ROS
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
    }

    move_group_interface.setPoseTarget(tf2::toMsg(T_B_arm_tool_link_pre));
    if(static_cast<bool>(move_group_interface.plan(plan))){
        move_group_interface.execute(plan);
    }
    else{
        // Shutdown ROS
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
    }

    // Move to grip
    move_group_interface.setPoseTarget(tf2::toMsg(T_B_arm_tool_link_gr));
    if(static_cast<bool>(move_group_interface.plan(plan))){
        move_group_interface.execute(plan);
    }
    else{
        // Shutdown ROS
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
    }

    // Catch the object
    move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.0305, 0.0305}));
    if(static_cast<bool>(move_group_interface_gripper.plan(plan))){
        move_group_interface_gripper.execute(plan);
    }
    else{
        // Shutdown ROS
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
    }
    move_group_interface.attachObject("green_cube_3::link");


    // Move 5cm above
    move_group_interface.setPoseTarget(tf2::toMsg(T_B_arm_tool_link_gr * Eigen::Translation3d(-0.1, 0, 0)));
    if(static_cast<bool>(move_group_interface.plan(plan))){
        move_group_interface.execute(plan);
    }
    else{
        // Shutdown ROS
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
    }

    // Detach object
    move_group_interface.detachObject("green_cube_3::link");
    move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.044, 0.04}));
    if(static_cast<bool>(move_group_interface_gripper.plan(plan))){
        move_group_interface_gripper.execute(plan);
    }
    else{
        // Shutdown ROS
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
    }
    move_group_interface.attachObject("green_cube_3", "arm_tool_link");

    RCLCPP_INFO(logger, "Finished all jobs! Exiting...");

    // Shutdown ROS
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}