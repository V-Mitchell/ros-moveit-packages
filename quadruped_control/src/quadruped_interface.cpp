/*  Author: Victor M
*   Email: victorcmitchell@gmail.com
*/

#include <chrono>
#include <thread>

#include "../include/quadruped_control/quadruped_kinematics.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Pose.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2_eigen/tf2_eigen.h>

int main (int argc, char* argv[]) {
    ros::init(argc, argv, "rover_arm_interface");
    ros::NodeHandle node_handle;

    static const std::string ROBOT_DESCRIPTION = "quadruped_robot";
    robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);

    quadruped_kinematics qKinematics(robot_model_loader.getModel());





}