#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include "./library/Controller.h"
#include "./library/MultiPathPlanner.h"


int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "pure_pursuit_controller");
    Controller controller;
    MultiPathPlanner planner;
    ROS_INFO("controller start");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}