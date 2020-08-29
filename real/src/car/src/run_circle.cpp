#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <iostream>

double lin_x = 0.0;

void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg) {
    lin_x = odomMsg->twist.twist.linear.x;
    if (fabs(lin_x) < 1e-2)
        lin_x = 0.0;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "run_circle");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher a_pub = n.advertise<geometry_msgs::Twist>("/can_run_circle", 1);
    
    ros::Subscriber sub = n.subscribe("/odom", 1, odomCB);
    

    geometry_msgs::PoseStamped goal_msg[4];
    for (int i = 0; i < 4; i++) {
        goal_msg[i].header.frame_id = "map";
        goal_msg[i].header.stamp = ros::Time::now();
        goal_msg[i].pose.orientation.x = 0.0;
        goal_msg[i].pose.orientation.y = 0.0;
        goal_msg[i].pose.orientation.z = 0.998;
        goal_msg[i].pose.orientation.w = -0.06;
    }
    goal_msg[0].pose.position.x = 3.0, goal_msg[0].pose.position.y = 0.0;
    goal_msg[1].pose.position.x = 4.5, goal_msg[1].pose.position.y = -2.1;
    goal_msg[2].pose.position.x = 0.0, goal_msg[2].pose.position.y = -2.6;
    goal_msg[3].pose.position.x = -0.5, goal_msg[3].pose.position.y = 0.0;

    int idx;
    geometry_msgs::Twist twist1, twist2;
    twist1.linear.x = 1.0;
    while (n.ok()) {
        std::cin >> idx;
        if (idx >= 0 && idx < 4)
            pub.publish(goal_msg[idx]);
        else if (idx < 0)
            a_pub.publish(twist1);
        else
            a_pub.publish(twist2);
    }

    return 0;
}