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

// Function to generate a figure-eight trajectory
nav_msgs::Path generate_figure_eight_trajectory(double radius, int points, std::string frame_id) {
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();

    for (int i = 0; i < points; i++) {
        double t = (double)i / points * 6 * M_PI; // parameter t goes from 0 to 2*PI
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id;

        // Figure-eight parametric equations
        pose.pose.position.x = radius * sin(t);
        pose.pose.position.y = radius * sin(t) * cos(t);
        pose.pose.position.z = 0;  // Assuming flat ground

        // Compute orientation towards the next point on the trajectory
        double next_t = (double)(i+1) / points * 2 * M_PI;
        double next_x = radius * sin(next_t);
        double next_y = radius * sin(next_t) * cos(next_t);
        double yaw = atan2(next_y - pose.pose.position.y, next_x - pose.pose.position.x);
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        path.poses.push_back(pose);
    }

    return path;
}

// Function to generate a wave trajectory
nav_msgs::Path generate_wave_trajectory(double amplitude, double wavelength, int points, std::string frame_id) {
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();

    // Wave along the x-axis
    for (int i = 0; i < points; i++) {
        double x = (double)i / points * wavelength;
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id;

        // Wave parametric equations
        pose.pose.position.x = x;
        pose.pose.position.y = amplitude * sin(2 * M_PI * x / wavelength);
        pose.pose.position.z = 0;  // Assuming flat ground

        // Compute orientation towards the next point on the trajectory
        double next_x = (double)(i + 1) / points * wavelength;
        double next_y = amplitude * sin(2 * M_PI * next_x / wavelength);
        double yaw = atan2(next_y - pose.pose.position.y, next_x - pose.pose.position.x);
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        path.poses.push_back(pose);
    }

    return path;
}

// Function to generate a multi-wave trajectory
nav_msgs::Path generate_multi_wave_trajectory(double amplitude, double wavelength, int waves, int points_per_wave, std::string frame_id) {
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();

    // Total points are points per wave multiplied by number of waves
    int total_points = points_per_wave * waves;
    
    // Extend the wave across multiple wavelengths
    for (int i = 0; i < total_points; i++) {
        double x = (double)i / points_per_wave * wavelength;
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id;

        // Multi-wave parametric equations
        pose.pose.position.x = x;
        pose.pose.position.y = amplitude * sin(2 * M_PI * x / wavelength);
        pose.pose.position.z = 0;  // Assuming flat ground

        // Compute orientation towards the next point on the trajectory
        double next_x = (double)(i + 1) / points_per_wave * wavelength;
        double next_y = amplitude * sin(2 * M_PI * next_x / wavelength);
        double yaw = atan2(next_y - pose.pose.position.y, next_x - pose.pose.position.x);
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

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
    double radius = 5.0;  // Radius of the circle
    double amplitude = 1.0;  // Amplitude of the waves
    double wavelength = 10.0; // Wavelength of the waves
    int waves = 5;           // Number of waves
    int points_per_wave = 300;  // Number of points in each wave
    int points = 1500;     // Number of points in the circle
    std::string frame_id = "map";  // Or the frame you want to use

    // Generate the trajectory
    //nav_msgs::Path path = generate_circle_trajectory(radius, points, frame_id);
    nav_msgs::Path path = generate_figure_eight_trajectory(radius, points, frame_id);
    //nav_msgs::Path path = generate_wave_trajectory(amplitude, wavelength, points, frame_id);
    //nav_msgs::Path path = generate_multi_wave_trajectory(amplitude, wavelength, waves, points_per_wave, frame_id);
    
    // Publish the trajectory
    trajectory_pub.publish(path);

    ros::spin();

    return 0;
}
