#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Geometry> 
#include <chrono>
#include <locale>
#include <tuple>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <boost/assert.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

extern "C" {
#include <math.h>
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h> //  close
#include <string.h> //  strerror
}


using namespace std;

static int data_length = 81;


boost::asio::serial_port* serial_port = 0;
static const uint8_t stop[6] = {0xA5, 0x5A, 0x04, 0x02, 0x06, 0xAA};
static const uint8_t mode[6] = {0xA5, 0x5A, 0x04, 0x01, 0x05, 0xAA};
static uint8_t data_raw[200];
static std::string name, frame_id;
static sensor_msgs::Imu msg;
static int fd_ = -1;
static ros::Publisher pub;
static uint8_t tmp[81];
static double degrees2rad = M_PI/180.0;

static float d2f_acc(uint8_t a[2]) {
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 16384.0f;
}

static float d2f_gyro(uint8_t a[2]) {
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 32.8f;
}

static float d2f_euler(uint8_t a[2]) {
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 10.0f;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "imu");
    ros::NodeHandle n("~");

    name = ros::this_node::getName();

    std::string port;
    if (n.hasParam("port"))
      n.getParam("port", port);
    else {
        ROS_ERROR("%s: must provide a port", name.c_str());
        return -1;
    }

    std::string model;
    if (n.hasParam("model"))
        n.getParam("model", model);
    else {
        ROS_ERROR("%s: must provide a model name", name.c_str());
        return -1;
    }
    ROS_WARN("Model set to %s", model.c_str()); 

    int baud;
    if (n.hasParam("baud"))
        n.getParam("baud", baud);
    else {
        ROS_ERROR("%s: must provide a baudrate", name.c_str());
        return -1;
    }
    ROS_WARN("Baudrate set to %d", baud);

    n.param("frame_id", frame_id, string("IMU_link"));

    double delay;
    n.param("delay", delay, 0.0);

    boost::asio::io_service io_service;
    serial_port = new boost::asio::serial_port(io_service);
    try {
        serial_port->open(port);
    } catch (boost::system::system_error &error) {
        ROS_ERROR("%s: Failed to open port %s with error %s",
                  name.c_str(), port.c_str(), error.what());
        return -1;
    }

    if (!serial_port->is_open()) {
        ROS_ERROR("%s: failed to open serial port %s",
                  name.c_str(), port.c_str());
        return -1;
    }

    typedef boost::asio::serial_port_base sb;

    sb::baud_rate baud_option(baud);
    sb::flow_control flow_control(sb::flow_control::none);
    sb::parity parity(sb::parity::none);
    sb::stop_bits stop_bits(sb::stop_bits::one);

    serial_port->set_option(baud_option);
    serial_port->set_option(flow_control);
    serial_port->set_option(parity);
    serial_port->set_option(stop_bits);

    const char *path = port.c_str();
    fd_ = open(path, O_RDWR);
    if(fd_ < 0) {    
        ROS_ERROR("Port Error!: %s", path);
        return -1;
    }

    pub = n.advertise<sensor_msgs::Imu>("data_raw", 1);
    if(model == "100D2") {
         write(fd_, stop, 6);
         usleep(1000 * 1000);
         write(fd_, mode, 6);
         usleep(1000 * 1000);
         data_length = 40;
    }

    // msg.linear_acceleration_covariance = *(new boost::array<double, 9>{ 0.04 , 0 , 0,
    //                                                                     0 , 0.04, 0,
    //                                                                     0 , 0 , 0.04});
    // msg.angular_velocity_covariance = *(new boost::array<double, 9>{0.02, 0 , 0,
    //                                                               0 , 0.02, 0,
    //                                                               0 , 0 , 0.02});
    msg.orientation_covariance = *(new boost::array<double, 9>{0.0025 , 0 , 0,
                                                              0, 0.0025, 0,
                                                              0, 0, 0.0025});

    int kk = 0;
    ROS_WARN("Streaming Data...");
    while (n.ok()) {
        read(fd_, tmp, sizeof(uint8_t) * data_length);
        memcpy(data_raw, tmp, sizeof(uint8_t) * data_length);

        bool found = false;
        for(kk = 0; kk < data_length - 1; ++kk) {
            if(model == "100D2" && data_raw[kk] == 0xA5 && data_raw[kk + 1] == 0x5A) {
                unsigned char *data = data_raw + kk;

                uint8_t data_length = data[2];

                uint32_t checksum = 0;
                for(int i = 0; i < data_length - 1  ; ++i)
                    checksum += (uint32_t) data[i+2];

                uint16_t check = checksum % 256;
                uint16_t check_true = data[data_length+1];

                if (check != check_true) {
                    printf("imu check error\n");
                    continue;
                }

                Eigen::Vector3d ea0(-d2f_euler(data + 3) * degrees2rad,
                                    -d2f_euler(data + 5) * degrees2rad,
                                    d2f_euler(data + 7) * degrees2rad);
                Eigen::Matrix3d R;
                R = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
                Eigen::Quaterniond q;
                q = R;
                msg.orientation.w = (double)q.w();
                msg.orientation.x = (double)q.x();
                msg.orientation.y = (double)q.y();
                msg.orientation.z = (double)q.z();

                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = frame_id;
                msg.angular_velocity.x = d2f_gyro(data + 17);
                msg.angular_velocity.y = -d2f_gyro(data + 15);
                msg.angular_velocity.z = d2f_gyro(data + 19);
                msg.linear_acceleration.x = -d2f_acc(data + 11) * 9.806;
                msg.linear_acceleration.y = d2f_acc(data + 9) * 9.806;
                msg.linear_acceleration.z = -d2f_acc(data + 13) * 9.806;
              
                pub.publish(msg);

                found = true;
            }
        }
    }

    // Stop continous and close device
    ROS_WARN("Wait 0.1s"); 
    ros::Duration(0.1).sleep();
    ::close(fd_);

    return 0;
}