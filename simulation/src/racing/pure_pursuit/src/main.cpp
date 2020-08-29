#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include "./library/Controller.h"
#include "./library/MultiPathPlanner.h"
#include "./library/Solver.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

struct Utils {
public:
    ros::NodeHandle nh;
    ros::Subscriber cal_nav_goals_sub, set_areas_sub;
    ros::Publisher nav_goals_pub, areas_pub;

    Utils() {
//   calculate goals queue by subscribe topic "/cal_nav_goals"
//   and send to topic "/nav_goals"
        cal_nav_goals_sub = nh.subscribe("/cal_nav_goals", 1,
                                         &Utils::cal_nav_goals_callback, this);
        nav_goals_pub = nh.advertise<std_msgs::Int32MultiArray>("/nav_goals", 1);


//    input 11 areas and publish to "/areas"
        set_areas_sub = nh.subscribe("/rviz/move_base_simple/goal", 1,
                                     &Utils::set_area_callback, this);
        areas_pub = nh.advertise<nav_msgs::Path>("/areas", 100);
    }

    void cal_nav_goals_callback(std_msgs::Int32MultiArray msg) {
//    calculate
        Solver solver;
        int s1, s2;
        ros::NodeHandle _nh("~");
        int tmp1;
        _nh.param("is_forward", tmp1, 0);
        if (tmp1 == 0) s2 = 2;
        else s2 = 9;
        ROS_ERROR("%d %d", s1, s2);
        solver.tmp_vec.push_back(s2);
        for (int i = 0; i + 1 < msg.data.size(); i++) {
            solver.mid_points.insert(msg.data[i]);
        }
        solver.goal = msg.data.back();
        solver.init();
        solver.dfs(s1, s2, 0);
        auto ans = solver.ans_vec;

//    publish to "/nav_goals"
        std_msgs::Int32MultiArray msg_pub;
        msg_pub.data = ans;
        nav_goals_pub.publish(msg_pub);
    }

    nav_msgs::Path areas;

    void set_area_callback(geometry_msgs::PoseStamped area) {
        areas.poses.push_back(area);
        ROS_INFO("%d area input", areas.poses.size());
        if (areas.poses.size() == 11) {
            ROS_INFO("get 11 areas input, pub to /areas");
            areas_pub.publish(areas);
        }
    }

    void set_init_callback(std_msgs::Float32MultiArray msg) {
        double x = msg.data[0], y = msg.data[1], yaw = msg.data[2];

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();

        pose.pose.position.x = x;
        pose.pose.position.y = y;

        pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }
};

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "pure_pursuit_controller");
    Controller controller;
    MultiPathPlanner planner;
    Utils utils;
    ROS_INFO("controller start");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}