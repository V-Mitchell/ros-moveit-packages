/*  Author: Victor M
*   Email: victorcmitchell@gmail.com
*/

#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Pose.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_eigen/tf2_eigen.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;
geometry_msgs::Pose set_end_effector_pose;
Eigen::Vector3d trans;

// Interface Joystick input to arm position and orientation
void inputCallback(const sensor_msgs::Joy& msg) {
    // Positon [m] Oreientation [rad]
    // Z Axis Control
    if (msg.buttons[2] == 1) set_end_effector_pose.position.z += 0.0025;
    if (msg.buttons[0] == 1) set_end_effector_pose.position.z -= 0.0025;

    // Y Axis Control
    if (msg.buttons[13] == 1) set_end_effector_pose.position.y -= 0.0025;
    if (msg.buttons[14] == 1) set_end_effector_pose.position.y += 0.0025;

    // Arm Base Yaw Control
    if (msg.buttons[4] == 1) set_end_effector_pose.orientation.z += 0.01;
    if (msg.buttons[5] == 1) set_end_effector_pose.orientation.z -= 0.01;

    // Gripper Fingers Open/Close
    if (msg.buttons[1] == 1) set_end_effector_pose.orientation.w -= 0.01; // Open
    if (msg.buttons[3] == 1 && set_end_effector_pose.orientation.w < 0) set_end_effector_pose.orientation.w += 0.01; // Close
    
    // Gripper Roll & Pitch Control
    set_end_effector_pose.orientation.y -= 0.01*msg.axes[3];
    set_end_effector_pose.orientation.x -= 0.01*msg.axes[4];
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rover_arm_interface");
    ros::NodeHandle node_handle;

    static const std::string PLANNING_GROUP = "arm";
    static const std::string GRIPPER_PLANNING_GROUP = "gripper";
    static const std::string ROBOT_DESCRIPTION = "robot_description"; // Taken from model loaded in move_group

    robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);

    // load robot model
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model Frame: %s", kinematic_model->getModelFrame().c_str());

    // get kinematic state of the robot
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    ROS_INFO("Arm set to default values...");

    const moveit::core::JointModelGroup* joint_model_group = 
        kinematic_model->getJointModelGroup(PLANNING_GROUP);
    ROS_INFO("Got Arm Joint Model Group: %s", PLANNING_GROUP.c_str());

    const moveit::core::JointModelGroup* gripper_model_group = 
        kinematic_model->getJointModelGroup(GRIPPER_PLANNING_GROUP);
    ROS_INFO("Got Gripper Model Group: %s", GRIPPER_PLANNING_GROUP.c_str());

    // setup MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    ROS_INFO("Setup Move Group Interface");

    moveit::planning_interface::MoveGroupInterface gripper_group_interface(GRIPPER_PLANNING_GROUP);
    ROS_INFO("Setup Gripper Move Group Interface");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    std::vector<double> joint_values;

    const std::vector<std::string>& gripper_joint_names = gripper_model_group->getVariableNames();
    std::vector<double> gripper_joint_values;

    // get initial joint values
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    kinematic_state->copyJointGroupPositions(gripper_model_group, gripper_joint_values);

    // Print out joint values
    ROS_INFO("Printing Joints");
    for (std::size_t i = 0; i < joint_names.size(); i++) {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    for (std::size_t i = 0; i < gripper_joint_names.size(); i++) {
        ROS_INFO("Joint %s: %f", gripper_joint_names[i].c_str(), gripper_joint_values[i]);
    }

    set_end_effector_pose = geometry_msgs::Pose(); // Initialize Pose datatype
    trans = Eigen::Vector3d(); // Initialize Vector3d datatype, used for Matrix Transformation operations

    // get initial pose of the end effector, return value: Eigen::Isometry3d
    Eigen::Isometry3d endPose = kinematic_state->getGlobalLinkTransform("arm_link5");
    Eigen::Isometry3d tmpPose; // Used for Matrix Transformations before calculating IK solution

    // Subscribe to control_input to recieve movement commands for arm
    ros::Subscriber sub = node_handle.subscribe("joy", 1000, inputCallback);
    // Publish joint positions from IK solution
    ros::Publisher joint_position_pub = node_handle.advertise<std_msgs::Float64MultiArray>("joint_positions", 1000);

    // Start ROS Spinning
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Visualization
    /*namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("arm_link5");
    visual_tools.deleteAllMarkers();
    // remote control allows users to step through code via buttons
    visual_tools.loadRemoteControl();*/

    // List all group of the robot
    ROS_INFO_NAMED("rover_arm_interface", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
        move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", \n"));

    // lower the allowed max velocity and acceleration
    move_group_interface.setMaxVelocityScalingFactor(1.0);
    move_group_interface.setMaxAccelerationScalingFactor(1.0);

    ROS_INFO("Beginning Arm Control Interface...");

    // delay
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    bool ik_found;

    while(ros::ok()) {
        tmpPose = endPose;
        trans[2] = set_end_effector_pose.position.z;
        trans[1] = set_end_effector_pose.position.y;
        tmpPose.translate(trans);

        ik_found = kinematic_state->setFromIK(joint_model_group, tmpPose, 0.1);
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        kinematic_state->copyJointGroupPositions(gripper_model_group, gripper_joint_values);

        joint_values[0] += set_end_effector_pose.orientation.z;
        joint_values[4] += set_end_effector_pose.orientation.y;
        joint_values[3] += set_end_effector_pose.orientation.x;

        gripper_joint_values[0] += set_end_effector_pose.orientation.w;
        gripper_joint_values[1] += set_end_effector_pose.orientation.w;
        gripper_joint_values[2] += set_end_effector_pose.orientation.w;

        move_group_interface.setJointValueTarget(joint_values);
        gripper_group_interface.setJointValueTarget(gripper_joint_values);
        move_group_interface.move();
        gripper_group_interface.move();

        ROS_INFO("End Effector Pose: XYZ[%.3f %.3f %.3f] RPY[%.3f %.3f %.3f] W[%.3f] %s",
            set_end_effector_pose.position.x, set_end_effector_pose.position.y, set_end_effector_pose.position.z,
            set_end_effector_pose.orientation.x, set_end_effector_pose.orientation.y, set_end_effector_pose.orientation.z,
            set_end_effector_pose.orientation.w, ik_found ? "" : "IK FAILED");


        std_msgs::Float64MultiArray msg;
        msg.data.resize(8);
        msg.data[0] = joint_values[0];
        msg.data[1] = joint_values[1];
        msg.data[2] = joint_values[2];
        msg.data[3] = joint_values[3];
        msg.data[4] = joint_values[4];
        msg.data[5] = gripper_joint_values[0];
        msg.data[6] = gripper_joint_values[1];
        msg.data[7] = gripper_joint_values[2];
        joint_position_pub.publish(msg); // Publish joint values

        // delay
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ros::shutdown();
    return 0;
}