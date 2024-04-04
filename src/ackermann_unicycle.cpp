#include <ackermann_unicycle/ackermann_unicycle.h>


AckermannUnicycle::AckermannUnicycle() : nh_("~"), x_(0), y_(0), z_(0), theta_(0) {
    
    this->velocity_ = 0.3;
    this->angular_velocity_ = 0;
    this->time = 0;

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_1", 50);
}

void AckermannUnicycle::publishOdometry() {
    double dt = 0.1; // time step
    this->time += dt;
    
    

    if (10 <= this->time && this->time <= 25) this->angular_velocity_= 0.2;
    else if (25 <= this->time && this->time <= 35)  this->angular_velocity_ = -0.2;
    else this->angular_velocity_ = 0.1;

    angular_velocity_ = this->angular_velocity_;


    // Update the pose of the robot
    x_ += velocity_ * cos(theta_) * dt;
    y_ += velocity_ * sin(theta_) * dt;
    z_ += 0; // assuming motion only in the x-y plane
    theta_ += angular_velocity_ * dt;
    this->theta_angle = theta_;
    // Publish odometry message

    this->odom_msg.header.stamp = ros::Time::now();
    this->odom_msg.header.frame_id = "map";
    this->odom_msg.child_frame_id = "map";

    // Position
    this->odom_msg.pose.pose.position.x = x_;
    this->odom_msg.pose.pose.position.y = y_;
    this->odom_msg.pose.pose.position.z = z_;

    // Orientation
    geometry_msgs::Quaternion odom_quat;
    odom_quat.x = 0.0;
    odom_quat.y = 0.0;
    odom_quat.z = sin(theta_ / 2.0);
    odom_quat.w = cos(theta_ / 2.0);
    odom_msg.pose.pose.orientation = odom_quat;

    // Twist
    odom_msg.twist.twist.linear.x = velocity_;
    odom_msg.twist.twist.angular.z = angular_velocity_;

    // Publish the message
    odom_pub_.publish(this->odom_msg);
}

nav_msgs::Odometry AckermannUnicycle::getLatestOdometry() {

    return this->odom_msg;
}

//#########################################################################à


TransformedOdometryPublisher::TransformedOdometryPublisher(AckermannUnicycle& ackermann_unicycle) : nh_("~"), ackermann_unicycle_(ackermann_unicycle) {
    transformed_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_2", 50);
}

void TransformedOdometryPublisher::publishTransformedOdometry() {
    // Get the latest odometry message from AckermannUnicycle
    nav_msgs::Odometry odom_msg = this->ackermann_unicycle_.getLatestOdometry();

    Eigen::Quaterniond pre_q;

    pre_q.x() = 0.0;
    pre_q.y() = 0.0;
    pre_q.z() = sin(this->ackermann_unicycle_.theta_angle / 2.0);
    pre_q.w() = cos(this->ackermann_unicycle_.theta_angle / 2.0);
    
    Eigen::Affine3d preRot;
    preRot.linear() = pre_q.toRotationMatrix();
    // Apply transformation to the odometry message
    // For demonstration, let's just apply a translation of 1 unit in the x-axis
    
    Eigen::Vector3d transl(0.1, -1.2, 0);

    Eigen::Quaterniond quat;
    quat.x() = 0;
    quat.y() = 0;
    quat.z() = 0;//sin(3.14 / 2.0);
    quat.w() = 1;//cos(3.14 / 2.0);


    Eigen::Affine3d transformation;
    transformation.translation() = preRot.linear() * transl;
    transformation.linear() = quat.toRotationMatrix();

    // Convert geometry_msgs::Pose to Eigen::Affine3d
    Eigen::Vector3d translation_fromMsg(odom_msg.pose.pose.position.x,
                                     odom_msg.pose.pose.position.y,
                                     odom_msg.pose.pose.position.z);
    Eigen::Quaterniond orientation_fromMsg(odom_msg.pose.pose.orientation.w,
                                   odom_msg.pose.pose.orientation.x,
                                   odom_msg.pose.pose.orientation.y,
                                   odom_msg.pose.pose.orientation.z);
    Eigen::Affine3d pose_affine;
    pose_affine.translation() = translation_fromMsg;
    pose_affine.linear() = orientation_fromMsg.toRotationMatrix();

    // Apply transformation
    Eigen::Affine3d transformed_pose_affine = transformation * pose_affine;

    // Convert back to geometry_msgs::Pose
    geometry_msgs::Pose transformed_pose_msg;
    transformed_pose_msg.position.x = transformed_pose_affine.translation().x();
    transformed_pose_msg.position.y = transformed_pose_affine.translation().y();
    transformed_pose_msg.position.z = transformed_pose_affine.translation().z();
    Eigen::Quaterniond transformed_orientation(transformed_pose_affine.rotation());
    transformed_pose_msg.orientation.w = transformed_orientation.w();
    transformed_pose_msg.orientation.x = transformed_orientation.x();
    transformed_pose_msg.orientation.y = transformed_orientation.y();
    transformed_pose_msg.orientation.z = transformed_orientation.z();

    // Update the odometry message with the transformed pose
    odom_msg.pose.pose = transformed_pose_msg;

    // Publish the transformed odometry message
    transformed_odom_pub_.publish(odom_msg);
}
