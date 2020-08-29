#include <ros/ros.h>
#include <iostream>
#include <boost/assert.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
extern "C" {
#include <fcntl.h>
}

using namespace std;
using namespace boost::asio;

serial_port *sp;
static int data_length = 2;
string name;
int steer_middle, steer_left, steer_right;
double steer_weight_left, steer_weight_right;

ros::Publisher debug_pub;

double left_p, right_p;

struct PID {
    double _kp = 1.2;
    double _kd;
    double output;

    double cal(double error) {
        double kp;
        
        if(fabs(error)<15)
            kp = _kp - 0.5;
        else if(fabs(error)<20)
            kp = _kp - 0.4;
        else if(fabs(error)<25)
            kp = _kp - 0.3;
        else if(fabs(error)<30)
            kp = _kp - 0.2;
        else
            kp = _kp;
        double out = kp * error;
        if (out > 50) out = 50;
        return out;
    }
} pid;

void TwistCallback(geometry_msgs::Twist twist) {
    // 转换弧度到角度
    twist.angular.z = twist.angular.z / 3.1415927 * 180;
    // 转换为 0 - 100
    twist.angular.z += 50;

    uint16_t angular = steer_middle;  // 0~100 映射到 440~1050
    // if (twist.angular.z < 50)
    //     angular = twist.angular.z * steer_weight_left + 400;
    // else if (twist.angular.z > 50)
    //     angular += (twist.angular.z - 50) * steer_weight_right;

    if (twist.angular.z < 0) twist.angular.z = 0;
    else if (twist.angular.z > 100) twist.angular.z = 100;

    geometry_msgs::Twist tmp;
    tmp.angular.z = fabs(twist.angular.z - 50);
    double output = pid.cal(fabs(tmp.angular.z - 50));

    if (twist.angular.z > 50)
        angular += right_p * fabs(twist.angular.z - 50) * steer_weight_right;
    else
        angular -= left_p * fabs(50 - twist.angular.z) * steer_weight_left;

    tmp.angular.y = output;
    debug_pub.publish(tmp);


    int8_t data[6];
    if(twist.linear.x >= 0.0)
        data[5] = 0x01;
    else
        data[5] = 0x00;

    int16_t speed = fabs(twist.linear.x) * 1000;   
    data[0] = 0x80;
    data[1] = speed/127;
    data[2] = speed%127;
    data[3] = angular/127;
    data[4] = angular%127;

    try {
        sp->write_some(buffer(data, 6));
       // ROS_WARN("sending data success...(encoder)");
    } catch (exception e) {
        ROS_ERROR(e.what());
        ROS_ERROR("sending data... error(encoder)");
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "car");
    ros::NodeHandle n("~");

    ros::Publisher encoder_pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/encoder_vel", 1);
    debug_pub = n.advertise<geometry_msgs::Twist>("/debug_pid", 1);
    ros::Subscriber sub = n.subscribe("/car/cmd_vel", 1, TwistCallback);

    n.param("steer_middle", steer_middle, 610);
    n.param("steer_left", steer_left, 370);
    n.param("steer_right", steer_right, 1200);
    steer_weight_left = (steer_middle - 370) / 50.0;
    steer_weight_right = (1200 - steer_middle) / 50.0;
    n.param("left_p", left_p, 1.0);
    n.param("right_p", right_p, 1.0);

    name = ros::this_node::getName();

    std::string port;
    n.param("car_port", port, string("/dev/car"));

    int baud;
    n.param("baud", baud, 115200);

    double pulse_ratio;
    n.param("pulse_ratio", pulse_ratio, 0.128083438);

    boost::asio::serial_port* serial_port = 0;
    boost::asio::io_service io_service;
    serial_port = new boost::asio::serial_port(io_service);
    try {
        serial_port->open(port);
    } catch (boost::system::system_error &error) {
        ROS_ERROR("%s: Failed to open port %s with error %s",
                 name.c_str(), port.c_str(), error.what());
        return -1;
    }

    boost::asio::io_service iosev;
    sp = new boost::asio::serial_port(iosev , port.c_str());
    sp->set_option(serial_port::baud_rate(baud));
    sp->set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp->set_option(serial_port::parity(serial_port::parity::none));
    sp->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp->set_option(serial_port::character_size(8));

    ROS_WARN("Encoder Streaming Data ...");

    geometry_msgs::TwistWithCovarianceStamped encoder_msg;
    encoder_msg.header.frame_id = "base_link";

    char data[data_length];
    int16_t encoder_pulse;
    while (n.ok()) {
        ros::spinOnce();
        try {
            sp->read_some(buffer(data, data_length));
            encoder_msg.header.stamp = ros::Time::now();
            encoder_pulse = data[0]*127+ data[1];
            encoder_msg.twist.twist.linear.x =  encoder_pulse * pulse_ratio;
            encoder_msg.twist.twist.linear.y += encoder_pulse;
            encoder_pub.publish(encoder_msg);
        } catch(exception e) {
            ROS_ERROR(e.what());
        }
    }

    ROS_WARN("Car Close, Wait 0.1s");
    ros::Duration(0.1).sleep();
    sp->close();

    return 0;
}