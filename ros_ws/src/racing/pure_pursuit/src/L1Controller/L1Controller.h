#include <iostream>
#include "ros/ros.h"
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
#include <math.h>
#include <std_srvs/Empty.h>


#define PI 3.14159265358979

/********************/
/* CLASS DEFINITION */
/********************/
class L1Controller
{
public:

    struct PID {
        double kp;
        double kd;
        double error;
        double error_last;
        double output;
    };

    enum MODE {
        BACK_STRAIGHT,
        BACK_LEFT,
        BACK_RIGHT,
        NORMAL
    };

    L1Controller();

    MODE mode = NORMAL;
    int back_cnt = 0;
   
    bool  isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos,const double lfw);
    double getYawFromPose(const geometry_msgs::Pose& carPose);        
    double getEta(const geometry_msgs::Pose& carPose);
    double getLfw();
    double getSteeringAngle(double eta);
    double getGasInput(double steeringAnge);
    bool getCurvature(double & center_x, double & center_y, double & radius);
    geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);

private:
    ros::NodeHandle n_;
    ros::Subscriber path_sub, goal_sub, scan_sub, amcl_sub, odom_sub;
    ros::Publisher cmd_vel_pub, moveBase_pub, eta_pub, eta_point_pub, curvature_pub;
    ros::Timer control_timer;
    tf::TransformListener tf_listener;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Point odom_goal_pos;
    nav_msgs::Odometry odom;
    nav_msgs::Path map_path;

    // for debug
    geometry_msgs::Twist eta_point;
   
    const double L = 0.335;

    double baseAngle, goalRadius;
    int controller_freq;
    bool foundForwardPt, goal_received, goal_reached;
    int tooCloseBack_cnt;

    double weight, eta_filter_limit;
    bool canSpeedChange;

    double baseSpeed;
    double min_speed, max_speed;
    double rush_speed, rush_dist;
    double acc_limit, acc_weight;
    double back_weight;
    bool useCurvature;
    double max_acc;
    
    bool canBack, canBackTurn;
    double backSpeed;

    double last_gasinput = 1.0;
    double Lfw; 
    bool canLfwChange;
    double Lfw_min, Lfw_max;
    
    PID pid;
    void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg);
    void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
    void laserCB(const sensor_msgs::LaserScan::ConstPtr& laserMsg);  
    void controlLoopCB(const ros::TimerEvent&);

}; // end of class