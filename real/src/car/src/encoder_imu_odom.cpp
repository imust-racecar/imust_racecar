#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

const double pi_ = 0.01745329252;

bool has_imu_data, has_encoder_data;
double th = 0, dth = 0;
double imu_dt = 0;
double vel_x = 0;

bool publish_tf;

void encoderCallback(geometry_msgs::TwistWithCovarianceStamped encoder_msg) {
    static ros::Time last_time = ros::Time::now();

    vel_x = encoder_msg.twist.twist.linear.x;

    last_time = ros::Time::now();
    has_encoder_data = true;
}

void imuCallback(sensor_msgs::Imu imu_msg) {
    static ros::Time last_time = ros::Time::now();
    
    imu_dt = (ros::Time::now() - last_time).toSec();

    if (fabs(imu_msg.angular_velocity.z) > 0.5)
        dth = imu_msg.angular_velocity.z * imu_dt;
    else
        dth = 0;

    last_time = ros::Time::now();
    has_imu_data = true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "encoder_imu_odom");
    ros::NodeHandle nh;

    ros::Subscriber encoder_sub = nh.subscribe("/encoder_vel", 1, encoderCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu/data_raw", 1, imuCallback);
    
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);

    nh.param("publish_tf", publish_tf, false);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.twist.twist.linear.y = 0, odom_msg.twist.twist.linear.z = 0;
    odom_msg.twist.twist.angular.x = 0, odom_msg.twist.twist.angular.y = 0;

    odom_msg.pose.covariance[0] = 0.01, odom_msg.pose.covariance[7] = 0.0, 
    odom_msg.pose.covariance[14] = 0.0, odom_msg.pose.covariance[21] = 0.0025,
    odom_msg.pose.covariance[25] = 0.0025, odom_msg.pose.covariance[28] = 0.0025;
    
    odom_msg.twist.covariance[0] = 0.05, odom_msg.twist.covariance[7] = 0.25,
    odom_msg.twist.covariance[14] = 0.01, odom_msg.twist.covariance[21] = 0.0,
    odom_msg.twist.covariance[25] = 0.0, odom_msg.twist.covariance[28] = 0.0;

    tf::TransformBroadcaster tf_broadcaster;
    
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";

    while (nh.ok()) {
        ros::spinOnce();
        if (!has_encoder_data || !has_imu_data) continue;
        has_encoder_data = false, has_imu_data = false;

        double dx = vel_x * imu_dt * cos(th * pi_);
        if (fabs(dx) < 0.05) dx = 0.0;
        double dy = vel_x * imu_dt * sin(th * pi_);
        if (fabs(dy) < 0.05) dy = 0.0;


        odom_msg.header.stamp = ros::Time::now();
        odom_msg.pose.pose.position.x += dx;
        odom_msg.pose.pose.position.y += dy;
        th += dth;
        odom_msg.twist.twist.linear.x = vel_x;
        odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th * pi_);
        odom_pub.publish(odom_msg);


        if (publish_tf) {
            tf_msg.header.stamp = ros::Time::now();
            tf_msg.transform.translation.x += dx;
            tf_msg.transform.translation.y += dy;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
            tf_broadcaster.sendTransform(tf_msg);
        }
    }

    return 0;
}