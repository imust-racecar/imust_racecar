//
// Created by r3v334 on 2020/8/21.
//

#ifndef ROS_WS_MULTIPATHPLANNER_H
#define ROS_WS_MULTIPATHPLANNER_H

#include "ros/ros.h"
#include "nav_msgs/GetPlan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32MultiArray.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "queue"
#include "vector"
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"

class MultiPathPlanner {
private:
//    ros system object
    ros::Publisher path_pub, start_status_pub;
    ros::Subscriber path_complete_sub, nav_goals_sub, odom_sub;
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener;
    ros::Timer path_timer;

private:
    struct Point {
        double x, y;
    };

    std::vector<Point> points = {
            {1e9, 1e9},
            {-7.4, 4.3},
            {-2.5, 4.3},
            {2.0, 3.4},
            {7.5, 4.3},
            {7.3, -0.7},
            {7.3, -4.3},
            {0.4, -4.2},
            {-4, -4.2},
            {-7.3, -4.1},
            {-1, -0.5},
            {1.2, 1.9}
    };

    std::queue<geometry_msgs::PoseStamped> goals_queue;
    nav_msgs::Odometry odom_msg;

    geometry_msgs::PoseStamped cur_goal;
    bool can_pub_path;
    ros::ServiceClient client;

public:
    MultiPathPlanner() {
        path_pub = nh_.advertise<nav_msgs::Path>("/path", 1);
        start_status_pub = nh_.advertise<std_msgs::Bool>("/start_status", 1);


        path_complete_sub = nh_.subscribe("/path_complete_status", 1,
                                          &MultiPathPlanner::path_complete_callback, this);
        nav_goals_sub = nh_.subscribe("/nav_goals", 1,
                                            &MultiPathPlanner::nav_goals_callback, this);
        odom_sub = nh_.subscribe("/odometry/filtered", 1,
                                    &MultiPathPlanner::odom_sub_callback, this);

        path_timer = nh_.createTimer(ros::Duration(1.0 / 10),
                                     &MultiPathPlanner::path_timer_callback, this);

        client = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    }

    void path_complete_callback(std_msgs::Float32 complete_status) {
//        if (complete_status.data < 50.0) {
//            if (goals_queue.size()) {
//                can_pub_path = true;
//                cur_goal = goals_queue.front();
//                goals_queue.pop();
//            } else {
//                can_pub_path = false;
//            }
//        }
    }

    void nav_goals_callback(std_msgs::Int32MultiArray goals) {
        for (int i = 0; i < goals.data.size(); i++) {
            geometry_msgs::PoseStamped goal;
            goal.header.frame_id = "map";
            goal.header.stamp = ros::Time::now();
            goal.pose.orientation.w = 1;
            goal.pose.position.x = points[goals.data[i]].x;
            goal.pose.position.y = points[goals.data[i]].y;
            goals_queue.push(goal);

        }
        cur_goal = goals_queue.front();
        goals_queue.pop();
        can_pub_path = true;
    }

    void path_timer_callback(const ros::TimerEvent&) {
        if (!can_pub_path) return;

        nav_msgs::GetPlan srv;
        tf::StampedTransform transform;
        try {
            tf_listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("Multi Path Planner lookupTransform error: %s", ex.what());
        }
        geometry_msgs::PoseStamped cur_pose;
        cur_pose.header.frame_id = "map";
        cur_pose.header.stamp = ros::Time::now();
        cur_pose.pose.orientation.w = 1;
        cur_pose.pose.position.x = transform.getOrigin().getX();
        cur_pose.pose.position.y = transform.getOrigin().getY();

        srv.request.start = cur_pose;
        srv.request.goal = cur_goal;

        if (client.call(srv)) {
            path_pub.publish(srv.response.plan);
            if (srv.response.plan.poses.size() < 30) {
                if (goals_queue.size()) {
                    cur_goal = goals_queue.front();
                    goals_queue.pop();
                    ROS_ERROR("next goal");
                }
            } else if (srv.response.plan.poses.size() < 10) {
                can_pub_path = false;
            }
        }
    }

    void odom_sub_callback(const nav_msgs::Odometry::ConstPtr & ptr) {
        odom_msg = *ptr;
    }
};


#endif //ROS_WS_MULTIPATHPLANNER_H
