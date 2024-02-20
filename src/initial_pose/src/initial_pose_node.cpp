#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // Process the initial pose information
    geometry_msgs::Pose pose = msg->pose.pose;
    ROS_INFO("Received initial pose: x=%f, y=%f, theta=%f", pose.position.x, pose.position.y, tf::getYaw(pose.orientation));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "initial_pose_subscriber");
    ros::NodeHandle nh;

    // Subscribe to the /initialpose topic
    ros::Subscriber initial_pose_subscriber = nh.subscribe("/initialpose", 1, initialPoseCallback);

    // Spin and process callbacks
    ros::spin();

    return 0;
}
