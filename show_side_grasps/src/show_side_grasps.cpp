#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/moveit_msgs/msg/grasp.hpp>
#include <thread>
#include <gazebo_msgs/gazebo_msgs/srv/get_entity_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


typedef enum {
    ROT_Z_90,
    ROT_Z_180,
    ROT_Z_270,
    ROT_Y_30,
    ROT_Y_MIN_30,
    ROT_Y_45

} ROTATIONS;


int main(int argc, char *argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
            "show_side_grasps",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("show_side_grasps");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Move group interfaces, planning scene interface setup
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm_torso");
    auto move_group_interface_gripper = MoveGroupInterface(node, "gripper");

    move_group_interface.setMaxVelocityScalingFactor(0.8);
    move_group_interface.setMaxAccelerationScalingFactor(0.6);

    move_group_interface_gripper.setMaxVelocityScalingFactor(0.8);
    move_group_interface_gripper.setMaxAccelerationScalingFactor(0.6);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Visual tools setup
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
            node, "base_footprint", rviz_visual_tools::RVIZ_MARKER_TOPIC,
            move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    // Get gripper model
    auto jmg = move_group_interface.getRobotModel()->getJointModelGroup("gripper");

    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client = node->create_client<gazebo_msgs::srv::GetEntityState>(
            "/get_entity_state");
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    Eigen::Isometry3d T_B_O;
    Eigen::Isometry3d T_B_table;
    Eigen::Isometry3d T_arm7_link_gr_O = Eigen::Translation3d(0.0, -0.001, 0.28) *
                                         Eigen::Quaternion(0.010816667244229508, -0.001105793243408453,
                                                           -0.7079713127308999, -0.7061574875912008).normalized();
    Eigen::Isometry3d T_arm7_link_pre_gr_O = Eigen::Translation3d(0.0, -0.001, 0.4) *
                                             Eigen::Quaternion(0.02604331807245956, -0.017432560531374066,
                                                               -0.707375654300129, -0.7061427158305615).normalized();

    Eigen::Isometry3d T_arm7_link_arm_tool_link =
            Eigen::Translation3d(0, 0, 0.046) * Eigen::Quaterniond(-0.5, 0.5, 0.5, 0.5).normalized();


    auto const prompt = [&moveit_visual_tools](auto text) {
        moveit_visual_tools.prompt(text);
    };


    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Main rotations
    std::map<ROTATIONS, Eigen::Isometry3d> T_O_Ogr_dict;

    T_O_Ogr_dict[ROT_Z_90] = Eigen::Isometry3d(Eigen::AngleAxisd(90.0/180.0*M_PI, Eigen::Vector3d::UnitZ()));
    T_O_Ogr_dict[ROT_Z_180] = Eigen::Isometry3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    T_O_Ogr_dict[ROT_Z_270] = Eigen::Isometry3d(Eigen::AngleAxisd(270.0/180.0*M_PI, Eigen::Vector3d::UnitZ()));

    T_O_Ogr_dict[ROT_Y_30] = Eigen::Isometry3d(Eigen::AngleAxisd(30.0/180.0*M_PI, Eigen::Vector3d::UnitY()));
    T_O_Ogr_dict[ROT_Y_MIN_30] = Eigen::Isometry3d(Eigen::AngleAxisd(-30.0/180.0*M_PI, Eigen::Vector3d::UnitY()));
    T_O_Ogr_dict[ROT_Y_45] = Eigen::Isometry3d(Eigen::AngleAxisd(45.0/180.0*M_PI, Eigen::Vector3d::UnitY()));





    RCLCPP_INFO(logger, "Computing position of arm_tool_link in base_footprint frame");
    Eigen::Isometry3d T_B_arm_tool_link_gr = T_B_O * T_arm7_link_gr_O.inverse() * T_arm7_link_arm_tool_link;
    Eigen::Isometry3d T_B_arm_tool_link_pre = T_B_O * T_arm7_link_pre_gr_O.inverse() * T_arm7_link_arm_tool_link;


    moveit_visual_tools.trigger();
    RCLCPP_INFO(logger, "Publishing frames...");

    // 1. Display gripper model
    moveit_msgs::msg::Grasp vis_gr;
    vis_gr.id = "grasp_1";
    vis_gr.grasp_pose.header.frame_id = "arm_tool_link";
    vis_gr.grasp_pose.pose = tf2::toMsg(T_B_arm_tool_link_gr);
    moveit_visual_tools.publishGrasps({vis_gr}, jmg);

    // 2. Display arm_tool_link pose during grip phase
    moveit_visual_tools.publishAxis(T_B_arm_tool_link_gr);

    // 3. Display arm_tool_link pose during pre grip phase
    moveit_visual_tools.publishAxis(T_B_arm_tool_link_pre);

    // 4. Display cube pose
    moveit_visual_tools.publishAxis(T_B_O);
    moveit_visual_tools.trigger();


// Execute the plan
    if (success) {
        moveit_visual_tools.trigger();
        prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
        moveit_visual_tools.trigger();
        move_group_interface.execute(plan);
    } else {
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Next step goes here

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}