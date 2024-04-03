#include <ros/ros.h>
#include <ackermann_unicycle/ackermann_unicycle.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ackermann_unicycle_node");

    // Initialize your AckermannUnicycle object
    AckermannUnicycle ackermann_unicycle;

    // Create a TransformedOdometryPublisher object with the AckermannUnicycle object
    TransformedOdometryPublisher transformed_odom_publisher(ackermann_unicycle);

    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok()) {
        // Publish odometry from AckermannUnicycle
        ackermann_unicycle.publishOdometry();

        // Publish transformed odometry
        transformed_odom_publisher.publishTransformedOdometry();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
