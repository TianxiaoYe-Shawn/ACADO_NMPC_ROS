#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher real_path_pub;
nav_msgs::Path path;

// Callback function for odometry messages
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg->header; // Copy timestamp and frame_id
    pose_stamped.pose = odom_msg->pose.pose; // Copy pose

    path.poses.push_back(pose_stamped);
    real_path_pub.publish(path);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_publisher");
    ros::NodeHandle nh;

    // Publisher for the path
    real_path_pub = nh.advertise<nav_msgs::Path>("estimated_state", 10, true);

    // Subscriber to odometry
    ros::Subscriber odom_sub = nh.subscribe("/odometry/filtered", 10, odomCallback);

    // Initialize path message
    path.header.frame_id = "odom";  // Make sure this is the correct frame

    ros::spin();

    return 0;
}

