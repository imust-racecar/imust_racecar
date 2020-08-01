#include "./L1Controller.h"

L1Controller::L1Controller() {
    //Controller parameter
    ros::NodeHandle pn("~");
    pn.param("controller_freq", controller_freq, 20);
    pn.param("baseAngle", baseAngle, 50.0);
    pn.param("goal_radius", goalRadius, 0.3);
    goalRadius = 0.3;

    // spped parameter
    pn.param("can_speed_change", canSpeedChange, true);
    pn.param("baseSpeed", baseSpeed, 2.5);
    pn.param("useCurvature", useCurvature, false);
    pn.param("min_speed", min_speed, 1.5);
    pn.param("max_speed", max_speed, 4.0);
    pn.param("rush_speed", rush_speed, baseSpeed);
    pn.param("rush_dist", rush_dist, 3.0);
    pn.param("acc_limit", acc_limit, 0.0);
    pn.param("acc_weight", acc_weight, 1.0);
    pn.param("back_weight", back_weight, 1.0);
    pn.param("max_acc", max_acc, 2.0);

    // back parameter
    pn.param("can_back", canBack, false);
    pn.param("back_speed", backSpeed, -1.0);
    pn.param("can_back_turn", canBackTurn, false);

    //eta parameter
    pn.param("eta_weight", weight, 0.9);

    // lfw parameter
    pn.param("can_Lfw_change", canLfwChange, false);
    pn.param("Lfw", Lfw, 1.2);
    pn.param("Lfw_min", Lfw_min, 0.0);
    pn.param("Lfw_max", Lfw_max, 1.9);
    
    // pid parameter
    pn.param("kp", pid.kp, 1.0);
    pn.param("kd", pid.kd, 1.0); 
    pid.error = 0;
    pid.error_last = 0;

    //Publishers and Subscribers
    path_sub = n_.subscribe("/move_base/GlobalPlanner/plan", 1, &L1Controller::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);
    scan_sub = n_.subscribe("/scan", 1, &L1Controller::laserCB, this);
    amcl_sub = n_.subscribe("/amcl_pose", 5, &L1Controller::amclCB, this);
    odom_sub = n_.subscribe("/odometry/filtered", 1, &L1Controller::odomCB, this);

    moveBase_pub = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    cmd_vel_pub = n_.advertise<geometry_msgs::Twist>("/car/cmd_vel", 1);
    curvature_pub = n_.advertise<geometry_msgs::Twist>("/car/curvature", 1);

    //Timer
    control_timer = n_.createTimer(ros::Duration((1.0)/controller_freq), &L1Controller::controlLoopCB, this); // Duration(0.05) -> 20Hz

    // Initialize the parameter
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;
    tooCloseBack_cnt = 0;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = baseAngle;
   

    //Show info
    ROS_INFO("[param] controller_freq: %d", controller_freq);
    ROS_INFO("[param] baseAngle: %f", baseAngle);
    ROS_INFO("[param] goal_radius: %f", goalRadius);
    ROS_INFO("-----------------------");
    ROS_INFO("[param] can_spped_change: %d", canSpeedChange);
    ROS_INFO("[param] baseSpeed: %f", baseSpeed);
    ROS_INFO("[param] min_speed", min_speed);
    ROS_INFO("[param] max_speed", max_speed);
    ROS_INFO("[param] rush_speed", rush_speed);
    ROS_INFO("[param] rush_dist", rush_dist);
    ROS_INFO("-----------------------");
    ROS_INFO("[param] can_back: %d", canBack);
    ROS_INFO("[param] back_speed: %f", backSpeed);
    ROS_INFO("-----------------------");
    ROS_INFO("[param] can_Lfw_change: %d", canLfwChange);
    ROS_INFO("[param] Lfw: %f", Lfw);
    ROS_INFO("[param] Lfw_min: %f", Lfw_min);
    ROS_INFO("[param] Lfw_max: %f", Lfw_max);
    ROS_INFO("-----------------------");
    ROS_INFO("[param] eta_weight: %f", weight);
    ROS_INFO("-----------------------");
    ROS_INFO("[param] acc_limit: %f", acc_limit);
    ROS_INFO("[param] acc_weight: %f", acc_weight);
    ROS_INFO("[param] back_weight: %f", back_weight);

    std::cout << "initiall is over " << std::endl;
}

void L1Controller::laserCB(const sensor_msgs::LaserScan::ConstPtr& laserMsg) {
    if (back_cnt > 0 || !canBack || !goal_received || goal_reached)
        return;
    int left = 0, right = 0;
    for (int i = 0; i < 40; i++)
        if (laserMsg->ranges[i] < 0.3)
            left++;
    for (int i = 320; i < laserMsg->ranges.size(); i++)
        if (laserMsg->ranges[i] < 0.3)
            right++;
    if (left + right >= 3) {
        mode = BACK_STRAIGHT;
        back_cnt = 35;
    }
}

void L1Controller::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg) {
    odom = *odomMsg;
}

void L1Controller::amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg) {
    if (this->goal_received) {
        double car2goal_x = this->odom_goal_pos.x - amclMsg->pose.pose.position.x;
        double car2goal_y = this->odom_goal_pos.y - amclMsg->pose.pose.position.y;
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
        if(dist2goal < this->goalRadius) {
            this->goal_reached = true;
            this->goal_received = false;
            ROS_INFO("Goal Reached !");
        }
    }
}

//接受到move_base规划的路径
void L1Controller::pathCB(const nav_msgs::Path::ConstPtr& pathMsg) {
    map_path = *pathMsg;      
}

//接受到rviz设置的目标
void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg) {
    try {
        /*将目标点转发给move_base*/
        moveBase_pub.publish(*goalMsg);       
        geometry_msgs::PoseStamped odom_goal;
        tf_listener.transformPose("odom", ros::Time(0) , *goalMsg, "map" ,odom_goal);
        odom_goal_pos = odom_goal.pose.position;
        goal_received = true;
        goal_reached = false;
    } catch(tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

double L1Controller::getYawFromPose(const geometry_msgs::Pose& carPose) {
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp,yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);

    return yaw;
}

bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos,const double lfw) {
    double dx = wayPt.x;
    double dy = wayPt.y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < lfw)
        return false;
    else if(dist >= lfw)
        return true;
}

//输入： 小车的当前位姿pose && move_base 规划的路径path
//输出： 小车下次应该走的point
geometry_msgs::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose) {
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose); //得到小车现在的 yaw 
    geometry_msgs::Point forwardPt;   
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;

    if(!goal_reached) {
        for(int i =0; i< map_path.poses.size(); i++) //遍历规划的路径
        {
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;
            try {
                tf_listener.transformPose("base_footprint", ros::Time(0) , map_path_pose, "map", odom_path_pose);
                odom_path_pose.pose.position.x += this->L / 2;
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                
                bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos, getLfw());
                if(_isWayPtAwayFromLfwDist) {
                    forwardPt = odom_path_wayPt;
                    foundForwardPt = true;
                    break;
                }
            } catch(tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        
    } else if(goal_reached) {
        geometry_msgs::PoseStamped odom_path_pose;
        geometry_msgs::PoseStamped map_path_pose = map_path.poses[0];
        map_path_pose.pose.position = odom_goal_pos;
        tf_listener.transformPose("base_footprint", ros::Time(0) , map_path_pose, "map", odom_path_pose);
        odom_path_pose.pose.position.x += this->L / 2;
        geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
        forwardPt = odom_path_wayPt;
        foundForwardPt = false;
        //ROS_INFO("goal REACHED!");
    }
    
    return forwardPt;
}

//输入： 小车的位姿
//输出： 
double L1Controller::getEta(const geometry_msgs::Pose& carPose) {
    static double last_eta = 0;
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

    double eta = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);

    return eta;
}

double L1Controller::getLfw() {
    if (!canLfwChange)
        return Lfw;

    double v = cmd_vel.linear.x;
    double lfw = ceil((v - 2.5)) / 0.5 * 0.1 + Lfw_min;

    if (lfw < Lfw_min) lfw = Lfw_min;
    else if (lfw > Lfw_max) lfw = Lfw_max;

    return lfw;
}

//using pid 
double L1Controller::getSteeringAngle(double eta) {
    return eta;

    static float diffrence=0.0,diffrence_last=0.0;
    float kp=0.0,kd=0.0;
    pid.error = -eta*(180.0/PI);
    diffrence = (pid.error-pid.error_last);
    pid.error_last = pid.error;
    if(diffrence > 10)
        diffrence = 10;
    else if(diffrence < -10)
        diffrence = -10;
    diffrence = 0.6*diffrence+0.4*diffrence_last;
    diffrence_last = diffrence;
    
    ROS_WARN("the error is %.2f, the error rate is %.2f",pid.error,diffrence);
    cmd_vel.linear.y = diffrence;

    //分段参数
    if(fabs(pid.error)<8.2)
        kp = pid.kp - 0.5;
    if(fabs(pid.error)<12)
        kp = pid.kp - 0.3;
    else if(fabs(pid.error)<16)
        kp = pid.kp - 0.2;
    else if(fabs(pid.error)<20)
        kp = pid.kp - 0.1;
    else
        kp = pid.kp;

    if(fabs(pid.error)<8 && fabs(diffrence)<3.0)
        kd = pid.kd - 1;            // -3
    else
        kd = pid.kd + 1;
    pid.output = pid.error*kp + diffrence*kd;
    
    return pid.output;
}

bool L1Controller::getCurvature(double & center_x, double & center_y, double & radius) {
    if (map_path.poses.size() < 7) {
        return false;
    }

    center_x = 0.0f;
    center_y = 0.0f;
    radius = 0.0f;

    double sum_x = 0.0f, sum_y = 0.0f;
    double sum_x2 = 0.0f, sum_y2 = 0.0f;
    double sum_x3 = 0.0f, sum_y3 = 0.0f;
    double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

    int N = map_path.poses.size() - 4;
    for (int i = 4; i < 40 + 4; i += 3) {
        geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
        geometry_msgs::PoseStamped car_path_pose;
        tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,car_path_pose);

        double x = car_path_pose.pose.position.x;
        double y = car_path_pose.pose.position.y;
        double x2 = x * x;
        double y2 = y * y;
        sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }

    double C, D, E, G, H;
    double a, b, c;

    C = N * sum_x2 - sum_x * sum_x;
    D = N * sum_xy - sum_x * sum_y;
    E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    G = N * sum_y2 - sum_y * sum_y;
    H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

    center_x = a / (-2);
    center_y = b / (-2);
    radius = sqrt(a * a + b * b - 4 * c) / 2;

    return true;
}

double L1Controller::getGasInput(double steeringAnge) {
    if (!canSpeedChange)
        return baseSpeed;
    if (map_path.poses.size() * 0.05 <= rush_dist)
        return rush_speed;

    static double curvature_last = 0.0;
    static bool flag_first_curv = true;

    double curvature = 0.0;
    geometry_msgs::Point car_path_wayPt, car_path_wayPt_last;
    bool flag_first = true;

    int size = 50;
    for(int i = 5; i < size; i++) {
        geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
        geometry_msgs::PoseStamped car_path_pose;
        try {
            tf_listener.transformPose("base_footprint", ros::Time(0) , map_path_pose, "map" ,car_path_pose);
            if(flag_first) {
                car_path_wayPt = car_path_pose.pose.position;                                   
                car_path_wayPt_last = car_path_wayPt;
                flag_first = false;
            } else{
                car_path_wayPt =  car_path_pose.pose.position;                                   
            }
        } catch(tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        curvature += fabs(car_path_wayPt.y - car_path_wayPt_last.y) * 2.0;
        
        car_path_wayPt_last = car_path_wayPt;
    }

    if(flag_first_curv) {
        flag_first_curv = false;
        curvature_last = curvature;
    }
    
    curvature = curvature*0.6 + curvature_last*0.4;
    curvature_last = curvature;
    
    ROS_WARN("the curvature of path is %.2f", curvature);

    //曲率联系速度
    double u = acc_limit - curvature; //curvature 是曲率(0~1.2) u是给定(0~4m/s)       //2.0
    
    if (u > 0)
        u *= acc_weight;
    if (u < 0)
        u *= back_weight;
    u += baseSpeed;

    if (u < min_speed) u = min_speed;
    else if (u > max_speed) u = max_speed;

    double delta = u - odom.twist.twist.linear.x;
    if (delta > max_acc) {
        u = max_acc + odom.twist.twist.linear.x;
        if (odom.twist.twist.linear.x < 2.0)
            u = odom.twist.twist.linear.x + max_acc * 0.5;
    }

    return u;
}

// timer for control
void L1Controller::controlLoopCB(const ros::TimerEvent&) {
    /*从里程计中得到位置信息，线速度信息*/
    geometry_msgs::Pose carPose = odom.pose.pose;  
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = baseAngle;

    if(goal_received) {
        /*Estimate Steering Angle*/
        if (mode == NORMAL) {
              //根据位姿计算出一个double
            double eta = getEta(carPose);
            cmd_vel.angular.z = getSteeringAngle(eta);
            cmd_vel.linear.x = getGasInput(1.0);
            if(foundForwardPt) {
                /*Estimate Gas Input*/
                if(!goal_reached) {     
                    last_gasinput = cmd_vel.linear.x;
                }
            } else {
                cmd_vel.linear.x = 0;
                ROS_ERROR("L1Controller cannot find forward point");    
            }
        } else {
            cmd_vel.linear.x = backSpeed;
            cmd_vel.angular.z = baseAngle;
            double eta = getEta(carPose);
            if (canBackTurn && foundForwardPt) {
                if (getSteeringAngle(eta) < 50) cmd_vel.angular.z = 0;
                else cmd_vel.angular.z = 100;
            }
            else
                cmd_vel.angular.z = baseAngle;
            --back_cnt;

            if (back_cnt == 0) 
                mode = NORMAL;
        }
    } else {
        ROS_WARN("not goal received");
    }

    ROS_INFO("\nGas = %.2f    Steering angle = %.2f     Lfw = %.2f", cmd_vel.linear.x, cmd_vel.angular.z, getLfw());
    cmd_vel_pub.publish(cmd_vel);
}
