#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/moveit_msgs/msg/grasp.hpp>
#include <thread>
#include <gazebo_msgs/gazebo_msgs/srv/get_entity_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include "../include/show_all_grasps/one_grasp_utils.hpp"


typedef enum {
    ROT_Y_IDENTITY,
    ROT_Y_90,
    ROT_Y_180,
    ROT_Y_270

} ROTATIONS;


typedef enum{
    ROT_Z_IDENTITY,
    ROT_Z_30,
    ROT_Z_MIN_30,
    ROT_Z_45

} SUB_ROTATIONS;


int main(int argc, char *argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
            "show_all_grasps",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("show_all_grasps");

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

    auto const prompt = [&moveit_visual_tools](auto text) {
        moveit_visual_tools.prompt(text);
    };

    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client = node->create_client<gazebo_msgs::srv::GetEntityState>(
            "/get_entity_state");
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    Eigen::Isometry3d T_B_O;
    Eigen::Isometry3d T_B_table;
    Eigen::Isometry3d T_arm7_link_gr_O = Eigen::Translation3d(0.0, -0.001, 0.28) * Eigen::Quaternion(0.010816667244229508, -0.001105793243408453, -0.7079713127308999, -0.7061574875912008).normalized();
    Eigen::Isometry3d T_arm7_link_pre_gr_O = Eigen::Translation3d(0.0, -0.001, 0.4) * Eigen::Quaternion(0.02604331807245956, -0.017432560531374066, -0.707375654300129, -0.7061427158305615).normalized();

    Eigen::Isometry3d T_arm7_link_arm_tool_link =
            Eigen::Translation3d(0, 0, 0.046) * Eigen::Quaterniond(-0.5, 0.5, 0.5, 0.5).normalized();


    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Get position of table with respect to base_footprint frame
    auto table_result = getItemPosition(logger, client, "table::link1", "base_footprint");
    if(table_result.has_value())
    {
        T_B_table = table_result.value();
    }
    else
    {
        shutdownExecutor(executor, spinner);
        return 0;
    }

    // Get position of green cube 3  with respect to base_footprint frame
    auto cube_result = getItemPosition(logger, client, "green_cube_3::link", "base_footprint");
    if(cube_result.has_value())
    {
        T_B_O = cube_result.value();
    }
    else
    {
        shutdownExecutor(executor, spinner);
        return 0;
    }

    // Main rotations
    std::map<ROTATIONS, Eigen::Isometry3d> T_O_Ogr_rotations_dict;
    std::map<SUB_ROTATIONS , Eigen::Isometry3d> T_O_Ogr_subrotations_dict;

    T_O_Ogr_rotations_dict[ROT_Y_IDENTITY] = Eigen::Isometry3d(Eigen::Isometry3d::Identity());
    T_O_Ogr_rotations_dict[ROT_Y_90] = Eigen::Isometry3d(Eigen::AngleAxisd(90.0/180.0*M_PI, Eigen::Vector3d::UnitY()));
    T_O_Ogr_rotations_dict[ROT_Y_180] = Eigen::Isometry3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
    T_O_Ogr_rotations_dict[ROT_Y_270] = Eigen::Isometry3d(Eigen::AngleAxisd(270.0/180.0*M_PI, Eigen::Vector3d::UnitY()));

    T_O_Ogr_subrotations_dict[ROT_Z_IDENTITY] = Eigen::Isometry3d(Eigen::Isometry3d::Identity());
    T_O_Ogr_subrotations_dict[ROT_Z_30] = Eigen::Isometry3d(Eigen::AngleAxisd(30.0/180.0*M_PI, Eigen::Vector3d::UnitZ()));
    T_O_Ogr_subrotations_dict[ROT_Z_MIN_30] = Eigen::Isometry3d(Eigen::AngleAxisd(-30.0/180.0*M_PI, Eigen::Vector3d::UnitZ()));

    std::vector<Eigen::Isometry3d> wall_rotations;
    wall_rotations.push_back(Eigen::Isometry3d(Eigen::Isometry3d::Identity()));
    wall_rotations.push_back(Eigen::Isometry3d(Eigen::AngleAxisd(90.0/180.0*M_PI, Eigen::Vector3d::UnitZ())));
    wall_rotations.push_back(Eigen::Isometry3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())));
    wall_rotations.push_back(Eigen::Isometry3d(Eigen::AngleAxisd(270.0/180.0*M_PI, Eigen::Vector3d::UnitZ())));

    wall_rotations.push_back(Eigen::Isometry3d(Eigen::AngleAxisd(90.0/180.0*M_PI, Eigen::Vector3d::UnitX())));
    wall_rotations.push_back(Eigen::Isometry3d(Eigen::AngleAxisd(-90.0/180.0*M_PI, Eigen::Vector3d::UnitX())));



    auto jmg = move_group_interface.getRobotModel()->getJointModelGroup("gripper");

    std::vector<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> T_E_gr_pre_vect;


    for(const auto& wall_rotation: wall_rotations){
        for(const auto& rot_pair: T_O_Ogr_rotations_dict){
            auto rotation = rot_pair.second;

            for(const auto& sub_rot_pair: T_O_Ogr_subrotations_dict){
                auto sub_rotation = sub_rot_pair.second;

                std::pair<Eigen::Isometry3d, Eigen::Isometry3d> grasp_pair =  {T_B_O * wall_rotation * rotation * sub_rotation * T_arm7_link_pre_gr_O.inverse() * T_arm7_link_arm_tool_link,
                                                                               T_B_O * wall_rotation * rotation * sub_rotation * T_arm7_link_gr_O.inverse() * T_arm7_link_arm_tool_link};

                T_E_gr_pre_vect.push_back(grasp_pair);
            }
        }
    }

    int i = 1;
    // Display grasps
    for(const auto& grasp_pair: T_E_gr_pre_vect){
        auto T_E_pre = grasp_pair.first;
        auto T_E_gr = grasp_pair.second;

        prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
        std::string grasp_numer_info_str = "Grasp number: " + std::to_string(i);
        RCLCPP_INFO(logger, grasp_numer_info_str.c_str());
        publishGrasp(moveit_visual_tools, jmg, T_B_O, T_E_pre, T_E_gr);
        moveit_visual_tools.deleteAllMarkers();
        ++i;
    }


    // Shutdown ROS
    shutdownExecutor(executor, spinner);
    return 0;
}