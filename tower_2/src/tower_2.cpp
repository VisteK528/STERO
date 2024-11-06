#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/moveit_msgs/msg/grasp.hpp>
#include <thread>
#include <gazebo_msgs/gazebo_msgs/srv/get_entity_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include "../include/tower_2/one_grasp_utils.hpp"
#include <random>
#include <functional>


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

constexpr double MAX_RADIUS = 0.75;
constexpr double GRIPPER_OPEN = 0.044;
constexpr double GRIPPER_CLOSED_3 = 0.03275;
constexpr double GRIPPER_CLOSED_2 = 0.02775;

using namespace std::literals::chrono_literals;

int main(int argc, char *argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
            "tower_2",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("tower_2");

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

    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client = node->create_client<gazebo_msgs::srv::GetEntityState>(
            "/get_entity_state");
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    Eigen::Isometry3d T_B_O_3;
    Eigen::Isometry3d T_B_O_2;
    Eigen::Isometry3d T_B_table;
    Eigen::Isometry3d T_arm7_link_gr_O = Eigen::Translation3d(0.0, -0.001, 0.28) * Eigen::Quaternion(0.010816667244229508, -0.001105793243408453, -0.7079713127308999, -0.7061574875912008).normalized();
    Eigen::Isometry3d T_arm7_link_pre_gr_O = Eigen::Translation3d(0.0, -0.001, 0.4) * Eigen::Quaternion(0.02604331807245956, -0.017432560531374066, -0.707375654300129, -0.7061427158305615).normalized();

    Eigen::Isometry3d T_arm7_link_arm_tool_link =
            Eigen::Translation3d(0, 0, 0.046) * Eigen::Quaterniond(-0.5, 0.5, 0.5, 0.5).normalized();


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
    auto green_cube_3_result = getItemPosition(logger, client, "green_cube_3::link", "base_footprint");
    if(green_cube_3_result.has_value())
    {
        T_B_O_3 = green_cube_3_result.value();
    }
    else
    {
        shutdownExecutor(executor, spinner);
        return 0;
    }

    // Get position of green cube 2  with respect to base_footprint frame
    auto green_cube_2_result = getItemPosition(logger, client, "green_cube_2::link", "base_footprint");
    if(green_cube_2_result.has_value())
    {
        T_B_O_2 = green_cube_2_result.value();
    }
    else
    {
        shutdownExecutor(executor, spinner);
        return 0;
    }


    // Add collision objects
    auto const table_collision_object = [frame_id =
    move_group_interface.getPlanningFrame(), table_pos = T_B_table] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "table";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 1.5;
        primitive.dimensions[primitive.BOX_Y] = 0.8;
        primitive.dimensions[primitive.BOX_Z] = 0.5;

        // Define the pose of the box (relative to the frame_id)
        Eigen::Isometry3d table_pos_fixed = table_pos * Eigen::Translation3d(0.0, 0.0, 0.235);
        geometry_msgs::msg::Pose box_pose = tf2::toMsg(table_pos_fixed);

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();


    auto green_cube_3_collision_object = [frame_id =
    move_group_interface.getPlanningFrame(), cube_pos = T_B_O_3] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "green_cube_3";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.07;
        primitive.dimensions[primitive.BOX_Y] = 0.07;
        primitive.dimensions[primitive.BOX_Z] = 0.07;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose = tf2::toMsg(cube_pos);

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();

    auto green_cube_2_collision_object = [frame_id =
    move_group_interface.getPlanningFrame(), cube_pos = T_B_O_2] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "green_cube_2";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.06;
        primitive.dimensions[primitive.BOX_Y] = 0.06;
        primitive.dimensions[primitive.BOX_Z] = 0.06;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose = tf2::toMsg(cube_pos);

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();

    planning_scene_interface.applyCollisionObject(table_collision_object);
    planning_scene_interface.applyCollisionObject(green_cube_3_collision_object);
    planning_scene_interface.applyCollisionObject(green_cube_2_collision_object);

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


    std::vector<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> T_E_cube_3_gr_pre_vect;
    std::vector<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> T_E_cube_2_gr_pre_vect;

    Eigen::Isometry3d T_B_tower_base = Eigen::Translation3d(0.7, 0.25, 0.54) * Eigen::Isometry3d(Eigen::AngleAxisd(-90.0/180.0*M_PI, Eigen::Vector3d::UnitX())) ;
    Eigen::Isometry3d T_B_tower_level1 = Eigen::Translation3d(0.7, 0.25, 0.61) * Eigen::Isometry3d(Eigen::AngleAxisd(-90.0/180.0*M_PI, Eigen::Vector3d::UnitX()));

    Eigen::Isometry3d T_B_arm_tool_link_pre_tower_base = T_B_tower_base * T_arm7_link_pre_gr_O.inverse() * T_arm7_link_arm_tool_link;
    Eigen::Isometry3d T_B_arm_tool_link_tower_base = T_B_tower_base * T_arm7_link_gr_O.inverse() * T_arm7_link_arm_tool_link;

    Eigen::Isometry3d T_B_arm_tool_link_pre_tower_level1 = T_B_tower_level1 * T_arm7_link_pre_gr_O.inverse() * T_arm7_link_arm_tool_link;
    Eigen::Isometry3d T_B_arm_tool_link_tower_level1 = T_B_tower_level1 * T_arm7_link_gr_O.inverse() * T_arm7_link_arm_tool_link;


    for(const auto& wall_rotation: wall_rotations){
        for(const auto& rot_pair: T_O_Ogr_rotations_dict){
            auto rotation = rot_pair.second;

            for(const auto& sub_rot_pair: T_O_Ogr_subrotations_dict){
                auto sub_rotation = sub_rot_pair.second;

                std::pair<Eigen::Isometry3d, Eigen::Isometry3d> green_cube_3_grasp_pair =  {
                        T_B_O_3 * wall_rotation * rotation * sub_rotation * T_arm7_link_pre_gr_O.inverse() * T_arm7_link_arm_tool_link,
                        T_B_O_3 * wall_rotation * rotation * sub_rotation * T_arm7_link_gr_O.inverse() * T_arm7_link_arm_tool_link};

                std::pair<Eigen::Isometry3d, Eigen::Isometry3d> green_cube_2_grasp_pair =  {
                        T_B_O_2 * wall_rotation * rotation * sub_rotation * T_arm7_link_pre_gr_O.inverse() * T_arm7_link_arm_tool_link,
                        T_B_O_2 * wall_rotation * rotation * sub_rotation * T_arm7_link_gr_O.inverse() * T_arm7_link_arm_tool_link};

                T_E_cube_3_gr_pre_vect.push_back(green_cube_3_grasp_pair);
                T_E_cube_2_gr_pre_vect.push_back(green_cube_2_grasp_pair);
            }
        }
    }

    // Move to the starting position
    MoveGroupInterface::Plan plan;
    move_group_interface.setJointValueTarget({0.350, 1.5708, 0.5585, 0, 0.5585, -1.5708, 1.3963, 0});
    if(static_cast<bool>(move_group_interface.plan(plan))){
        move_group_interface.execute(plan);
    }

    // Open gripper
    move_group_interface_gripper.setJointValueTarget(std::vector<double>({GRIPPER_OPEN, GRIPPER_OPEN}));
    if(static_cast<bool>(move_group_interface_gripper.plan(plan))){
        move_group_interface_gripper.execute(plan);
    }
    else{
        shutdownExecutor(executor, spinner);
        return 0;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(0, T_E_cube_3_gr_pre_vect.size() - 1);

    // Vector to store the elements that match the criteria
    std::vector<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> matchedElements;

    std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan> pre_grasp_plan_pair;
    std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan> grasp_plan_pair;

    pre_grasp_plan_pair.first = false;
    grasp_plan_pair.first = false;


    // Loop to pick elements randomly until we find 3 that match the criteria
    while (matchedElements.size() < 1) {
        int random_index = distr(gen);
        std::pair<Eigen::Isometry3d, Eigen::Isometry3d> random_element = T_E_cube_3_gr_pre_vect[random_index];

        // Check if the element matches the criteria
        bool from_top = random_element.first.translation().z() > random_element.second.translation().z() + 0.02;
        bool pre_grasp_range = std::sqrt(std::pow(random_element.first.translation().x(), 2) + std::pow(random_element.first.translation().y(), 2)) <= MAX_RADIUS;
        bool grasp_range = std::sqrt(std::pow(random_element.second.translation().x(), 2) + std::pow(random_element.second.translation().y(), 2)) <= MAX_RADIUS;

        if (from_top && pre_grasp_range && grasp_range) {
            move_group_interface.setPoseTarget(tf2::toMsg(random_element.first));
            pre_grasp_plan_pair.first = static_cast<bool>(move_group_interface.plan(pre_grasp_plan_pair.second));

            move_group_interface.setPoseTarget(tf2::toMsg(random_element.second));
            grasp_plan_pair.first = static_cast<bool>(move_group_interface.plan(grasp_plan_pair.second));

            if(grasp_plan_pair.first && pre_grasp_plan_pair.first){
                matchedElements.push_back(random_element);
            }
        }
    }


    if(grasp_plan_pair.first && pre_grasp_plan_pair.first){
        RCLCPP_INFO(logger, "Success!!! Grabbing the cube!");

        moveit_visual_tools.publishAxis(T_B_O_3);
        moveit_visual_tools.publishAxis(T_B_tower_base);
        moveit_visual_tools.publishAxis(matchedElements[0].first);
        moveit_visual_tools.publishAxis(matchedElements[0].second);

        moveit_visual_tools.publishAxis(T_B_arm_tool_link_tower_base);
        moveit_visual_tools.publishAxis(T_B_arm_tool_link_pre_tower_base);
        moveit_visual_tools.trigger();


        move_group_interface.setPoseTarget(tf2::toMsg(matchedElements[0].first));
        if(static_cast<bool>(move_group_interface.plan(plan))){
            move_group_interface.execute(plan);
        }
        else{
            shutdownExecutor(executor, spinner);
            return 0;
        }

        move_group_interface.setPoseTarget(tf2::toMsg(matchedElements[0].second));
        if(static_cast<bool>(move_group_interface.plan(plan))){
            move_group_interface.execute(plan);
        }
        else{
            shutdownExecutor(executor, spinner);
            return 0;
        }

        planning_scene_interface.removeCollisionObjects({"green_cube_3"});
        move_group_interface.setMaxVelocityScalingFactor(0.4);
        move_group_interface.setMaxAccelerationScalingFactor(0.2);

        move_group_interface_gripper.setMaxVelocityScalingFactor(0.1);
        move_group_interface_gripper.setMaxAccelerationScalingFactor(0.1);

        // Catch the object
        move_group_interface_gripper.setJointValueTarget(std::vector<double>({GRIPPER_CLOSED_3, GRIPPER_CLOSED_3}));
        if(static_cast<bool>(move_group_interface_gripper.plan(plan))){
            move_group_interface_gripper.execute(plan);
        }
        else{
            shutdownExecutor(executor, spinner);
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        move_group_interface.setMaxVelocityScalingFactor(0.8);
        move_group_interface.setMaxAccelerationScalingFactor(0.6);
        move_group_interface_gripper.setMaxVelocityScalingFactor(0.8);
        move_group_interface_gripper.setMaxAccelerationScalingFactor(0.6);

        std::vector<std::string> touch_links = {"gripper_left_finger_link", "gripper_right_finger_link"};
        move_group_interface.setPoseTarget(tf2::toMsg(Eigen::Translation3d(0.0, 0.0, 0.1) * matchedElements[0].second));
        if(static_cast<bool>(move_group_interface.plan(plan))){
            // Attach cube
            planning_scene_interface.addCollisionObjects({green_cube_3_collision_object});
            move_group_interface.attachObject("green_cube_3", "arm_7_link", touch_links);

            // Execute move
            move_group_interface.execute(plan);
        }
        else{
            shutdownExecutor(executor, spinner);
            return 0;
        }

        // Move cube to the tower base
        move_group_interface.setPoseTarget(tf2::toMsg(T_B_arm_tool_link_pre_tower_base));
        if(static_cast<bool>(move_group_interface.plan(plan))){
            move_group_interface.execute(plan);
        }
        else{
            shutdownExecutor(executor, spinner);
            return 0;
        }

        move_group_interface.setPoseTarget(tf2::toMsg(T_B_arm_tool_link_tower_base));
        if(static_cast<bool>(move_group_interface.plan(plan))){
            move_group_interface.execute(plan);
        }
        else{
            shutdownExecutor(executor, spinner);
            return 0;
        }

        // Detach object
        move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.044, 0.04}));
        if(static_cast<bool>(move_group_interface_gripper.plan(plan))){
            // Detach the cube
            move_group_interface.detachObject({"green_cube_3"});

            // Execute the move
            move_group_interface_gripper.execute(plan);
        }
        else{
            shutdownExecutor(executor, spinner);
            return 0;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // Move to the start pos
        move_group_interface.setJointValueTarget({0.350, 1.5708, 0.5585, 0, 0.5585, -1.5708, 1.3963, 0});
        if(static_cast<bool>(move_group_interface.plan(plan))){
            move_group_interface.execute(plan);
        }


    }
    else{
        RCLCPP_ERROR(logger, "Planning failed, exiting...");
    }

    std::random_device rd2;
    std::mt19937 gen2(rd2());
    std::uniform_int_distribution<> distr2(0, T_E_cube_2_gr_pre_vect.size() - 1);

    // Vector to store the elements that match the criteria
    std::vector<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> matchedElements2;
    pre_grasp_plan_pair.first = false;
    grasp_plan_pair.first = false;


    // Loop to pick elements randomly until we find 3 that match the criteria
    while (matchedElements2.size() < 1) {
        int random_index = distr2(gen2);
        std::pair<Eigen::Isometry3d, Eigen::Isometry3d> random_element = T_E_cube_2_gr_pre_vect[random_index];

        // Check if the element matches the criteria
        bool from_top = random_element.first.translation().z() > random_element.second.translation().z() + 0.02;
        bool pre_grasp_range = std::sqrt(std::pow(random_element.first.translation().x(), 2) + std::pow(random_element.first.translation().y(), 2)) <= MAX_RADIUS;
        bool grasp_range = std::sqrt(std::pow(random_element.second.translation().x(), 2) + std::pow(random_element.second.translation().y(), 2)) <= MAX_RADIUS;

        if (from_top && pre_grasp_range && grasp_range) {
            move_group_interface.setPoseTarget(tf2::toMsg(random_element.first));
            pre_grasp_plan_pair.first = static_cast<bool>(move_group_interface.plan(pre_grasp_plan_pair.second));

            move_group_interface.setPoseTarget(tf2::toMsg(random_element.second));
            grasp_plan_pair.first = static_cast<bool>(move_group_interface.plan(grasp_plan_pair.second));

            if(grasp_plan_pair.first && pre_grasp_plan_pair.first){
                matchedElements2.push_back(random_element);
            }
        }
    }


    if(grasp_plan_pair.first && pre_grasp_plan_pair.first){
        RCLCPP_INFO(logger, "Success!!! Grabbing the cube!");

        moveit_visual_tools.publishAxis(T_B_tower_level1);
        moveit_visual_tools.publishAxis(matchedElements2[0].first);
        moveit_visual_tools.publishAxis(matchedElements2[0].second);

        moveit_visual_tools.publishAxis(T_B_arm_tool_link_tower_level1);
        moveit_visual_tools.publishAxis(T_B_arm_tool_link_pre_tower_level1);
        moveit_visual_tools.trigger();


        move_group_interface.setPoseTarget(tf2::toMsg(matchedElements2[0].first));
        if(static_cast<bool>(move_group_interface.plan(plan))){
            move_group_interface.execute(plan);
        }
        else{
            shutdownExecutor(executor, spinner);
            return 0;
        }

        move_group_interface.setPoseTarget(tf2::toMsg(matchedElements2[0].second));
        if(static_cast<bool>(move_group_interface.plan(plan))){
            move_group_interface.execute(plan);
        }
        else{
            shutdownExecutor(executor, spinner);
            return 0;
        }

        planning_scene_interface.removeCollisionObjects({"green_cube_2"});
        move_group_interface.setMaxVelocityScalingFactor(0.4);
        move_group_interface.setMaxAccelerationScalingFactor(0.2);

        move_group_interface_gripper.setMaxVelocityScalingFactor(0.1);
        move_group_interface_gripper.setMaxAccelerationScalingFactor(0.1);

        // Catch the object
        move_group_interface_gripper.setJointValueTarget(std::vector<double>({GRIPPER_CLOSED_2, GRIPPER_CLOSED_2}));
        if(static_cast<bool>(move_group_interface_gripper.plan(plan))){
            move_group_interface_gripper.execute(plan);
        }
        else{
            shutdownExecutor(executor, spinner);
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        move_group_interface.setMaxVelocityScalingFactor(0.8);
        move_group_interface.setMaxAccelerationScalingFactor(0.6);
        move_group_interface_gripper.setMaxVelocityScalingFactor(0.8);
        move_group_interface_gripper.setMaxAccelerationScalingFactor(0.6);

        std::vector<std::string> touch_links = {"gripper_left_finger_link", "gripper_right_finger_link"};
        move_group_interface.setPoseTarget(tf2::toMsg(Eigen::Translation3d(0.0, 0.0, 0.1) * matchedElements2[0].second));
        if(static_cast<bool>(move_group_interface.plan(plan))){
            // Attach cube
            planning_scene_interface.addCollisionObjects({green_cube_2_collision_object});
            move_group_interface.attachObject("green_cube_2", "arm_7_link", touch_links);

            // Execute move
            move_group_interface.execute(plan);
        }
        else{
            shutdownExecutor(executor, spinner);
            return 0;
        }

        // Move cube to the tower base
        move_group_interface.setPoseTarget(tf2::toMsg(T_B_arm_tool_link_pre_tower_level1));
        if(static_cast<bool>(move_group_interface.plan(plan))){
            move_group_interface.execute(plan);
        }
        else{
            shutdownExecutor(executor, spinner);
            return 0;
        }

        move_group_interface.setPoseTarget(tf2::toMsg(T_B_arm_tool_link_tower_level1));
        if(static_cast<bool>(move_group_interface.plan(plan))){
            move_group_interface.execute(plan);
        }
        else{
            shutdownExecutor(executor, spinner);
            return 0;
        }

        // Detach object
        move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.044, 0.04}));
        if(static_cast<bool>(move_group_interface_gripper.plan(plan))){
            // Detach the cube
            move_group_interface.detachObject({"green_cube_2"});

            // Execute the move
            move_group_interface_gripper.execute(plan);
        }
        else{
            shutdownExecutor(executor, spinner);
            return 0;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // Move to the start pos
        move_group_interface.setJointValueTarget({0.350, 1.5708, 0.5585, 0, 0.5585, -1.5708, 1.3963, 0});
        if(static_cast<bool>(move_group_interface.plan(plan))){
            move_group_interface.execute(plan);
        }


    }
    else{
        RCLCPP_ERROR(logger, "Planning failed, exiting...");
    }



    // Shutdown ROS
    shutdownExecutor(executor, spinner);
    return 0;
}