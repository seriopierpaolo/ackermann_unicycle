#include <ackermann_unicycle/ackermann_unicycle.h>
#include <random> // Include this for random number generation


AckermannUnicycle::AckermannUnicycle() : nh_("~"), x_(0), y_(0), z_(0), theta_(0) {
    
    this->velocity_ = 1;
    this->angular_velocity_ = 0;
    //this->time = 0;

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_1", 1);

    this->acc_clock = 0;
    this->acc_switch = 0;

    this->noise_magnitude = 0.2;
}

void AckermannUnicycle::publishOdometry() {
    double dt = 0.1; // time step
    
    //this->time += dt;

    this->acc_clock += dt;
    
    std::srand( (unsigned)time( NULL ) );

    //ACCELERATION CONTROLLER
    if (this->acc_clock > 1.0 && int(this->acc_clock) % 10 == 0) {
        this->acc_switch = !this->acc_switch;
        this->acc_clock = 0;
        }
    
    //std::cout << this->acc_switch << std::endl;

    if (this->acc_switch) this->angular_velocity_= 0.6*(float) rand()/RAND_MAX;
    else this->angular_velocity_= - 0.8*(float) rand()/RAND_MAX ;
    

    //if (this->acc_switch) this->angular_velocity_= 0.2;
    //else this->angular_velocity_= -0.2 ;

    //std::cout<<theta_<<std::endl;
    angular_velocity_ = this->angular_velocity_;


    // Update the pose of the robot
    x_ += velocity_ * cos(theta_) * dt;
    y_ += velocity_ * sin(theta_) * dt;
    z_ += 0; // assuming motion only in the x-y plane
    theta_ += angular_velocity_ * dt;
    this->theta_angle = theta_;
    // Publish odometry message

    // Add white noise to the position
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-this->noise_magnitude, this->noise_magnitude);
    double noise_x = distribution(generator);
    double noise_y = distribution(generator);

    this->odom_msg.header.stamp = ros::Time::now();
    this->odom_msg.header.frame_id = "map";
    this->odom_msg.child_frame_id = "map";

    /*
    // Position
    this->odom_msg.pose.pose.position.x = x_;
    this->odom_msg.pose.pose.position.y = y_;
    this->odom_msg.pose.pose.position.z = z_;
    */

    // Position with noise
    this->odom_msg.pose.pose.position.x = x_ + noise_x;
    this->odom_msg.pose.pose.position.y = y_ + noise_y;
    this->odom_msg.pose.pose.position.z = z_;

    // Orientation
    geometry_msgs::Quaternion odom_quat;
    odom_quat.x = 0.0;
    odom_quat.y = 0.0;
    odom_quat.z = sin(theta_ / 2.0); //odom_quat.z = sin(theta_ / 2.0);
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
    transformed_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_2", 1);
}

void TransformedOdometryPublisher::publishTransformedOdometry() {
    // Get the latest odometry message from AckermannUnicycle
    nav_msgs::Odometry odom_msg = this->ackermann_unicycle_.getLatestOdometry();
    
    Eigen::Vector3d des_transl( 0.132, 1.557, 0.0);
/*
    Eigen::Quaterniond des_quat;
    des_quat.x() = 0;
    des_quat.y() = 0;
    des_quat.z() = -0.9880316;//sin(3.14 / 2.0);
    des_quat.w() = 0.1542514;//cos(3.14 / 2.0);
*/
    Eigen::Quaterniond des_quat;
    des_quat.x() = 0;
    des_quat.y() = 0;
    des_quat.z() = 0;//sin(3.14 / 2.0);
    des_quat.w() = 1;//cos(3.14 / 2.0);

    Eigen::Affine3d des_transf;
    des_transf.translation() = des_transl;
    des_transf.linear() = des_quat.toRotationMatrix();

    // Convert geometry_msgs::Pose to Eigen::Affine3d
    Eigen::Vector3d translation_fromMsg(odom_msg.pose.pose.position.x,
                                        odom_msg.pose.pose.position.y,
                                        odom_msg.pose.pose.position.z);

    Eigen::Quaterniond orientation_fromMsg( odom_msg.pose.pose.orientation.w,
                                            odom_msg.pose.pose.orientation.x,
                                            odom_msg.pose.pose.orientation.y,
                                            odom_msg.pose.pose.orientation.z);
    Eigen::Affine3d pose_affine;
    pose_affine.translation() = translation_fromMsg;
    pose_affine.linear() = orientation_fromMsg.toRotationMatrix();

    // Apply transformation
    Eigen::Affine3d transformed_pose_affine = pose_affine;

    transformed_pose_affine = des_transf.inverse() * pose_affine * des_transf;

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


    // Add white noise to the transformed position
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-this->ackermann_unicycle_.noise_magnitude, this->ackermann_unicycle_.noise_magnitude);
    double noise_x = distribution(generator);
    double noise_y = distribution(generator);
    
    transformed_pose_msg.position.x += noise_x;
    transformed_pose_msg.position.y += noise_y;
    // Update the odometry message with the transformed pose
    odom_msg.pose.pose = transformed_pose_msg;

    // Publish the transformed odometry message
    transformed_odom_pub_.publish(odom_msg);
}
