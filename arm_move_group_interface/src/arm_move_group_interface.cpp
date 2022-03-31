/*  Author: Victor M
*   Last Modified: yyyy-mm-dd 
*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

/* #include <moveit_visual_tools/moveit_visual_tools.h>  This has not been ported to ros2 yet */
#include <rviz_visual_tools/rviz_visual_tools.hpp>
/* this is a standin for moveit_visual_tools visual_tools.prompt */
#include <moveit/macros/console_colors.h>

// prompt called to stall program until key press
void prompt(const std::string& message) {
    printf(MOVEIT_CONSOLE_COLOR_GREEN "\n%s" MOVEIT_CONSOLE_COLOR_RESET, message.c_str());
    fflush(stdout);
    while (std::cin.get() != '\n' && rclcpp::ok());
}

static const rclcpp::Logger LOGGER = rclcpp::get_logger("arm_move_group");

int main (int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("arm_move_group_interface", node_options);

    // Spin up a SingleThreadedExecutor for the current state monitor for the robot's state
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() {executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "arm";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    /*const moveit::core::JointModelGroup* joint_model_group = 
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);*/


    // Visualization
    namespace rvt = rviz_visual_tools;
    rviz_visual_tools::RvizVisualTools visual_tools("arm_link0", "arm_move_group", move_group_node);
    visual_tools.deleteAllMarkers();

    // Text marker
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "RoverManipulatorInterface", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), 
        std::ostream_iterator<std::string>(std::cout, ", "));

    // start the demo with a prompt
    prompt("Press 'Enter' to start the demo");

    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.5;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.0;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();
    prompt("Press 'Enter' to continue the demo");

    // Move to a pose goal


    // END_TUTORIAL
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    rclcpp::shutdown();
    return 0;
}