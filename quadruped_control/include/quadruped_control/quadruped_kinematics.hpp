/*  Author: Victor M
*   Email: victorcmitchell@gmail.com
*/

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Pose.h"


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

//#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_eigen/tf2_eigen.h>

#ifndef QUADRUPED_KINEMATICS
#define QUADRUPED_KINEMATICS

class quadruped_kinematics {
    public:
        enum LEG_ID { FL , FR , BL , BR , X };

        quadruped_kinematics(const moveit::core::RobotModelPtr& kinematic_model) {
            this->kinematic_model = kinematic_model;
            kinematic_state = moveit::core::RobotStatePtr(new moveit::core::RobotState(kinematic_model));
            const std::vector<moveit::core::JointModelGroup *> groups = kinematic_model->getJointModelGroups();

            // Initialize data type for each leg
            for (std::size_t i = 0; i < groups.size(); i++) {
                leg_t leg;
                leg.leg_model_group = groups[i];
                leg.group_name = groups[i]->getName();
                leg.id = getId(leg.group_name);
                if (leg.id == LEG_ID::X) throw std::invalid_argument("Leg Identity Unknown");
                leg.leg_group_interface = moveit::planning_interface::MoveGroupInterface(leg.group_name);
                kinematic_state->copyJointGroupPositions(leg.leg_model_group, leg.joint_values);

                legs.insert( std::pair<LEG_ID, leg_t>(leg.id, leg) );
            }
            RPY = Eigen::Vector3d();
            XYZ = Eigen::Vector3d();
        }

        std::string toString() {
            std::string stringRep;

            return stringRep;
        }

        void setLegPositionTargets(const std::map<LEG_ID, Eigen::Vector3d>& legPositions) {
            if (legPositions.size() < 4) throw std::invalid_argument("Invalid Argument Received");
            // iterate trhough positions map setting position targets
            for (std::map<LEG_ID, Eigen::Vector3d>::const_iterator it = legPositions.begin();
                it != legPositions.end(); ++it) {
                    double x = it->second[0];
                    double y = it->second[1];
                    double z = it->second[2];
                    legs[it->first].leg_group_interface.setPositionTarget(x, y, z);
            }
        }

        void setLegGaitPositionTargets(double t) {

        }

        void getJointValuesFromPositionTargets(const std::map<LEG_ID, Eigen::Vector3d>& legPositions, std::vector<double>& jointValues) {

        }

        void getJointValuesFromGaitTargets(std::vector<double>& jointValues) {

        }

        void executeLegTargets() {
            // iterate through each leg and execute any set targets
            for (std::map<LEG_ID, leg_t>::iterator it = legs.begin();
                it != legs.end(); ++it) {
                    it->second.leg_group_interface.move();
                }
        }

    private:
        struct leg_t {
            LEG_ID id;
            std::string group_name;
            moveit::core::JointModelGroup* leg_model_group;
            moveit::planning_interface::MoveGroupInterface leg_group_interface;
            std::vector<double> joint_values;

            // Leg Transformation Matrix
            Eigen::Matrix4d Tx;
        };

        moveit::core::RobotModelPtr kinematic_model;
        moveit::core::RobotStatePtr kinematic_state;
        std::map<LEG_ID, leg_t> legs;

        Eigen::Vector3d RPY; // Roll, Pitch, Yaw
        Eigen::Vector3d XYZ; // Robot Body XYZ Offset to Legs

        LEG_ID getId(std::string name) {
            if (name.find("fl_") != std::string::npos) {
                return LEG_ID::FL;
            } else if (name.find("fr_") != std::string::npos) {
                return LEG_ID::FR;
            } else if (name.find("bl_") != std::string::npos) {
                return LEG_ID::BL;
            } else if (name.find("br_") != std::string::npos) {
                return LEG_ID::BR;
            }
            return LEG_ID::X;
        }

        Eigen::Matrix3d Rx(double roll) {
            Eigen::Matrix3d R = Eigen::Matrix3d(Eigen::AngleAxisd(roll, Eigen::Vector3d(1, 0, 0)));
            return R;
        }

        Eigen::Matrix3d Ry(double yaw) {
            Eigen::Matrix3d R = Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d(0, 1, 0)));
            return R;
        }

        Eigen::Matrix3d Rz(double pitch) {
            Eigen::Matrix3d R = Eigen::Matrix3d(Eigen::AngleAxisd(pitch, Eigen::Vector3d(0, 0, 1)));
            return R;
        }

        Eigen::Matrix4d Tm(double roll, double pitch, double yaw, Eigen::Vector3d X) {
            Eigen::Matrix4d T;
            T.setIdentity();
            T.block<3,3>(0,0) = Rx(roll) * Ry(yaw) * Rz(pitch);
            T.block<3,1>(0,3) = X;
            return T;
        }


};

#endif