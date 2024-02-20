#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // Process the initial pose information
    geometry_msgs::Pose pose = msg->pose.pose;
    ROS_INFO("Received initial pose: x=%f, y=%f, theta=%f", pose.position.x, pose.position.y, tf::getYaw(pose.orientation));
}

void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Process the goal pose information
    geometry_msgs::Pose pose = msg->pose;
    ROS_INFO("Received goal pose: x=%f, y=%f, theta=%f", pose.position.x, pose.position.y, tf::getYaw(pose.orientation));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_posehandler");
    ros::NodeHandle ip_n;
    ros::NodeHandle gp_n;

    // Subscribe to the /initialpose topic
    ros::Subscriber initial_pose_subscriber = ip_n.subscribe("/initialpose", 1, initialPoseCallback);
    ros::Subscriber goal_pose_subscriber = gp_n.subscribe("/move_base_simple/goal", 1, goalPoseCallback);

    // Spin and process callbacks
    ros::spin();

    return 0;
}
