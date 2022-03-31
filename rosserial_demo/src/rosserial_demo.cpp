/*  Author: Victor M
*   Email: victorcmitchell@gmail.com
*/

#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "rosserial_demo/JointPositions.h"


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "rosserial_demo");
    ros::NodeHandle nh;

    //ros::Publisher publisher = nh.advertise<std_msgs::Float32MultiArray>("test", 100);
    ros::Publisher publisher = nh.advertise<rosserial_demo::JointPositions>("test", 100);

    // Start ROS Spinning
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok()) {
        //std_msgs::Float32MultiArray msg;
        //msg.data.resize(6);
        rosserial_demo::JointPositions joint_msg;

        /*for (std::size_t i = 0; i < 6; i++) {
            msg.data[i] = i;
        }
        publisher.publish(msg);*/
        joint_msg.joint0 = 1.0;
        joint_msg.joint1 = 2.0;
        joint_msg.joint2 = 3.0;
        joint_msg.joint3 = 4.0;
        joint_msg.joint4 = 5.0;
        joint_msg.joint5 = 6.0;
        publisher.publish(joint_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    ros::shutdown();
    return 0;
}