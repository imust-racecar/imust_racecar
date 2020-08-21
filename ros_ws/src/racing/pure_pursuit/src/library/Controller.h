//
// Created by r3v334 on 2020/8/21.
//

#ifndef ROS_WS_CONTROLLER_H
#define ROS_WS_CONTROLLER_H

#endif //ROS_WS_CONTROLLER_H

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <cmath>

class Controller {
private:
//    speed params
    bool speed_change;
    double base_speed, min_speed, max_speed;
//    acceleration params
    double acc_limit, max_acc;
    double brake_weight, gas_weight;
//    lfw params
    bool lfw_change;
    double base_lfw, min_lfw, max_lfw;
//    control params
    double goal_radius;
    double control_frequency;
    std::string path_topic;

private:
//    ros system object
    ros::NodeHandle n_;
    ros::Timer control_loop_timer, path_complete_timer;
    ros::Publisher path_complete_pub, cmd_vel_pub;
    ros::Subscriber path_sub, start_sub;

private:
//    data object
    nav_msgs::Path path_msg;
    tf::TransformListener tf_listener;
    bool can_start = false;


public:
    Controller() {
        ros::NodeHandle pn("~");
//        speed params
        pn.param("speed_change", speed_change, false);
        pn.param("base_speed", base_speed, 2.0);
        pn.param("min_speed", min_speed, 2.0);
        pn.param("max_speed", max_speed, 2.0);
//        acceleration params
        pn.param("acc_limit", acc_limit, 0.3);
        pn.param("max_acc", max_acc, 2.0);
        pn.param("brake_weight", brake_weight, 1.0);
        pn.param("gas_weight", gas_weight, 1.0);
//        lfw params
        pn.param("lfw_change", lfw_change, false);
        pn.param("base_lfw", base_lfw, 0.6);
        pn.param("min_lfw", min_lfw, 0.6);
        pn.param("max_lfw", max_lfw, 0.6);
//        control params
        pn.param("control_frequency", control_frequency, 20.0);
        pn.param("goal_radius", goal_radius, 0.3);
        pn.param("path_topic", path_topic, std::string("/move_base/GlobalPlanner/plan"));


//        initialize ros system object
        control_loop_timer = n_.createTimer(ros::Duration(1.0 / control_frequency),
                                            &Controller::control_loop_callback, this);
        path_complete_timer = n_.createTimer(ros::Duration(1.0 / control_frequency),
                                             &Controller::path_complete_callback, this);
        path_complete_pub = n_.advertise<std_msgs::Float32>("/path_complete_status", 1);
        cmd_vel_pub = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        path_sub = n_.subscribe(path_topic, 1,
                                &Controller::path_sub_callback, this);
        start_sub = n_.subscribe("/start_status", 1,
                                 &Controller::start_sub_callback, this);

//        log info to show the initial params
        ROS_WARN("speed_change: %d\n", speed_change);
        ROS_WARN("base_speed: %.2f\n", base_speed);
        ROS_WARN("min_speed: %.2f\n", min_speed);
        ROS_WARN("max_speed: %.2f\n", max_speed);
        ROS_WARN("------------------\n");
        ROS_WARN("acc_limit: %.2f\n", acc_limit);
        ROS_WARN("max_acc: %.2f\n", max_acc);
        ROS_WARN("brake_weight: %.2f\n", brake_weight);
        ROS_WARN("gas_weight: %.2f\n", gas_weight);
        ROS_WARN("------------------\n");
        ROS_WARN("lfw_change: %d\n", lfw_change);
        ROS_WARN("base_lfw: %.2f\n", base_lfw);
        ROS_WARN("min_lfw: %.2f\n", min_lfw);
        ROS_WARN("max_lfw: %.2f\n", max_lfw);
        ROS_WARN("------------------\n");
        ROS_WARN("control_frequency: %.2f\n", control_frequency);
        ROS_WARN("goal_radius: %.2f\n", goal_radius);
        ROS_WARN("path_topic: %s\n", path_topic.c_str());
    }

public:
    void control_loop_callback(const ros::TimerEvent &) {
        geometry_msgs::Twist cmd;
        if (can_start) {
            cmd.linear.x = get_gas_input();
            cmd.angular.z = get_steering_input();
        } else {
            cmd.linear.x = 0;
            cmd.angular.z = 0;
        }
        ROS_WARN("gas: %.2f, steering: %.2f\n", cmd.linear.x, cmd.angular.z);
        cmd_vel_pub.publish(cmd);
    }

    void path_complete_callback(const ros::TimerEvent &) {
        std_msgs::Float32 msg;
        msg.data = (int) path_msg.poses.size() - get_closest_point_index();
        path_complete_pub.publish(msg);
        if (msg.data * 0.05 < goal_radius)
            can_start = false;
    }

    void path_sub_callback(const nav_msgs::Path::ConstPtr &ptr) {
        path_msg = *ptr;
    }

    void start_sub_callback(const std_msgs::Bool::ConstPtr & ptr) {
        can_start = ptr->data;
    }

private:
    geometry_msgs::PoseStamped transform_from_map_to_car(const geometry_msgs::PoseStamped data) {
        geometry_msgs::PoseStamped pose;
        try {
            tf_listener.transformPose("base_footprint", ros::Time(0) , data, "map", pose);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("pure_pursuit transform error: %s", ex.what());
        }
        return pose;
    }

    int get_closest_point_index() {
        int index = 0;
        double dis = 1e9;
        for (int i = 0; i < path_msg.poses.size(); i++) {
            auto pose = transform_from_map_to_car(path_msg.poses[i]);
            auto x = pose.pose.position.x;
            auto y = pose.pose.position.y;
            auto tmp_dis = x * x + y * y;
            if (tmp_dis < dis) {
                dis = tmp_dis;
                index = i;
            }
        }
        return index;
    }

    double get_lfw() {
        return base_lfw;
    }

    double get_gas_input() {
        if (!speed_change) return base_speed;

        double delta = 0;
        double last_y;
        for (int i = get_closest_point_index(), j = 0; i < path_msg.poses.size() && j < 40; i++, j++) {
            auto pose = transform_from_map_to_car(path_msg.poses[i]);
            auto y = pose.pose.position.y;
            if (j == 0) last_y = y;
            delta += fabs(y - last_y);
            last_y = y;
        }
        auto u = acc_limit - delta;
        if (u > 0) u *= gas_weight;
        else if (u < 0) u *= brake_weight;
        u += base_speed;
        if (u > max_speed) u = max_speed;
        else if (u < min_speed) u = min_speed;

        return u;
    }

    double get_steering_input() {
        geometry_msgs::PoseStamped pose;
        for (int i = get_closest_point_index(); i < path_msg.poses.size(); i++) {
            pose = transform_from_map_to_car(path_msg.poses[i]);
            auto x = pose.pose.position.x;
            auto y = pose.pose.position.y;
            if (x * x + y * y >= get_lfw() * get_lfw()) {
                break;
            }
        }
        return std::atan2(pose.pose.position.y, pose.pose.position.x);
    }
};
