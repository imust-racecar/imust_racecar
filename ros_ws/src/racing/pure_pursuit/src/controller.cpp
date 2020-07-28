#include <ros/ros.h>
#include "./L1Controller/L1Controller.h"

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "L1Controller");
    L1Controller controller;
    ROS_INFO("controller start");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}