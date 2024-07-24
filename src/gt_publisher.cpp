#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_listener.h>

ros::Publisher real_path_pub;
nav_msgs::Path path;
std::string model_name = "jackal";  
tf::TransformListener* tf_listener;

// Callback function for model states messages
void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& model_states_msg) {
    auto it = std::find(model_states_msg->name.begin(), model_states_msg->name.end(), model_name);
    if (it != model_states_msg->name.end()) {
        int index = std::distance(model_states_msg->name.begin(), it);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "world";  
        pose_stamped.pose = model_states_msg->pose[index];

        geometry_msgs::PoseStamped transformed_pose;
        try {
            tf_listener->transformPose("map", pose_stamped, transformed_pose);
            
            path.poses.push_back(transformed_pose);
            path.header.stamp = ros::Time::now();
            path.header.frame_id = "map";  // RViz frame_id
            real_path_pub.publish(path);
        } catch (tf::TransformException &ex) {
            ROS_WARN("Transform error: %s", ex.what());
        }
    } else {
        ROS_WARN("Model: %s not found in the current model states.", model_name.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gt_publisher");
    ros::NodeHandle nh;

    tf_listener = new tf::TransformListener();

    // Publisher for the path
    real_path_pub = nh.advertise<nav_msgs::Path>("model_trajectory", 10, true);

    // Subscriber to model states
    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback);

    // Initialize path message
    path.header.frame_id = "map"; 

    ros::spin();

    delete tf_listener;
    return 0;
}

