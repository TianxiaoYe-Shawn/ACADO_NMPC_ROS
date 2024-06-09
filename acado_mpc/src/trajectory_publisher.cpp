#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

// Function to generate a circle trajectory
nav_msgs::Path generate_circle_trajectory(double radius, int points, std::string frame_id) {
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();

    for (int i = 0; i < points; i++) {
        double angle = i * 6 * M_PI / points;
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id;
        pose.pose.position.x = radius * cos(angle);
        pose.pose.position.y = radius * sin(angle);
        pose.pose.position.z = 0;  // Assuming flat ground
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

        path.poses.push_back(pose);
    }

    return path;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle nh;

    ros::Publisher trajectory_pub = nh.advertise<nav_msgs::Path>("/trajectory", 10, true);

    // Parameters for the circle trajectory
    double radius = 10.0;  // Radius of the circle
    int points = 3000;     // Number of points in the circle
    std::string frame_id = "odom";  // Or the frame you want to use

    // Generate the trajectory
    nav_msgs::Path path = generate_circle_trajectory(radius, points, frame_id);

    // Publish the trajectory
    trajectory_pub.publish(path);

    ros::spin();

    return 0;
}
