#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
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

    move_group_interface.setMaxVelocityScalingFactor(0.8);
    move_group_interface.setMaxAccelerationScalingFactor(0.6);

    move_group_interface_gripper.setMaxVelocityScalingFactor(0.8);
    move_group_interface_gripper.setMaxAccelerationScalingFactor(0.6);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
            node, "base_footprint", rviz_visual_tools::RVIZ_MARKER_TOPIC,
            move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client = node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    Eigen::Isometry3d T_B_O;
    Eigen::Isometry3d T_B_table;
    Eigen::Isometry3d T_arm7_link_pre_gr_O = Eigen::Translation3d(0.0028227705405636633, 0.008221876185005018, 0.4088518719276147) * Eigen::Quaternion(0.02604331807245956, -0.017432560531374066, -0.707375654300129, -0.7061427158305615).normalized();
    Eigen::Isometry3d T_arm7_link_gr_O = Eigen::Translation3d(0.0023609, 0.0070378, 0.2781856) * Eigen::Quaternion(0.010816667244229508, -0.001105793243408453, -0.7079713127308999, -0.7061574875912008).normalized();

    Eigen::Isometry3d T_arm7_link_arm_tool_link = Eigen::Translation3d(0, 0, 0.046) * Eigen::Quaterniond(-0.5, 0.5, 0.5, 0.5).normalized();

    // Get position of cube and table from the world in base_footprint frame
    RCLCPP_INFO(logger, "Getting position of table in base_footprint frame...");
    request->name = "table::link1";
    request->reference_frame = "base_footprint";
    while(!client->wait_for_service(std::chrono::seconds(1s))){
        if(!rclcpp::ok()){
            RCLCPP_ERROR(logger, "Interrupted. Exiting.");
            return 0;
        }
        RCLCPP_INFO(logger, "service not available, waiting...");
    }

    auto future_result1 = client->async_send_request(request);
    if(future_result1.wait_for(3s) == std::future_status::ready){
        auto resp = future_result1.get();
        tf2::fromMsg(resp->state.pose, T_B_table);
    }else{
        RCLCPP_ERROR(logger, "Failed to call service /get_entity_state");

        // Shutdown ROS
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
    }


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

    // Check if object is inside specified workspace
    Eigen::Vector3d cube_position = T_B_O.translation();
    const double max_radius = 0.75;

    if(std::pow(cube_position.x(), 2) + std::pow(cube_position.y(), 2) > max_radius){
        RCLCPP_ERROR(logger, "Cube is too far from the robot base and thus cannot be reached! Exiting...");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
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


    auto cube_collision_object = [frame_id =
    move_group_interface.getPlanningFrame(), cube_pos = T_B_O] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "cube";
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

    planning_scene_interface.applyCollisionObject(table_collision_object);
    planning_scene_interface.applyCollisionObject(cube_collision_object);

    // Move to the starting position
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group_interface.setJointValueTarget({0.350, 1.5708, 0.5585, 0, 0.5585, -1.5708, 1.3963, 0});
    if(static_cast<bool>(move_group_interface.plan(plan))){
        move_group_interface.execute(plan);
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

    planning_scene_interface.removeCollisionObjects({"cube"});
    move_group_interface.setMaxVelocityScalingFactor(0.4);
    move_group_interface.setMaxAccelerationScalingFactor(0.2);

    move_group_interface_gripper.setMaxVelocityScalingFactor(0.1);
    move_group_interface_gripper.setMaxAccelerationScalingFactor(0.1);

    // Catch the object
    move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.034, 0.034}));
    if(static_cast<bool>(move_group_interface_gripper.plan(plan))){
        move_group_interface_gripper.execute(plan);
    }
    else{
        // Shutdown ROS
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
    }

    std::vector<std::string> touch_links = {"gripper_left_finger_link", "gripper_right_finger_link"};

    // Move 5cm above
    move_group_interface.setPoseTarget(tf2::toMsg(T_B_arm_tool_link_gr * Eigen::Translation3d(-0.1, 0, 0)));
    if(static_cast<bool>(move_group_interface.plan(plan))){
        planning_scene_interface.addCollisionObjects({cube_collision_object});
        move_group_interface.attachObject("cube", "arm_7_link", touch_links);
        move_group_interface.execute(plan);
    }
    else{
        // Shutdown ROS
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
    }

    // Move 5cm left
    move_group_interface.setPoseTarget(tf2::toMsg(T_B_arm_tool_link_gr * Eigen::Translation3d(-0.1, 0, 0) * Eigen::Translation3d(0, 0, 0.2)));
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
    move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.044, 0.04}));
    if(static_cast<bool>(move_group_interface_gripper.plan(plan))){
        move_group_interface.detachObject({"cube"});
        move_group_interface_gripper.execute(plan);
    }
    else{
        // Shutdown ROS
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
    }

    RCLCPP_INFO(logger, "Finished all jobs! Exiting...");

    // Shutdown ROS
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}