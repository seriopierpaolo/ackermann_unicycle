#ifndef ACKERMANN_UNICYCLE_H
#define ACKERMANN_UNICYCLE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>


class AckermannUnicycle {
public:
    AckermannUnicycle();

    void publishOdometry();
    nav_msgs::Odometry getLatestOdometry(); // Assuming this method exists

    nav_msgs::Odometry odom_msg;
    double theta_angle;

private:
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;

    //double time;
    double x_, y_, z_, theta_; // robot pose
    double velocity_, angular_velocity_; // control inputs
    bool acc_switch; //angular acceleration sign controller
    double acc_clock;
    
};



class TransformedOdometryPublisher {
public:
    TransformedOdometryPublisher(AckermannUnicycle& ackermann_unicycle);

    void publishTransformedOdometry();

private:
    ros::NodeHandle nh_;
    ros::Publisher transformed_odom_pub_;

    AckermannUnicycle& ackermann_unicycle_;
};

#endif // ACKERMANN_UNICYCLE_H

