#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Process the goal pose information
    geometry_msgs::Pose pose = msg->pose;
    ROS_INFO("Received goal pose: x=%f, y=%f, theta=%f", pose.position.x, pose.position.y, tf::getYaw(pose.orientation));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_goalpose");
    ros::NodeHandle n;

    // Subscribe to the /move_base_simple/goal topic
    ros::Subscriber goal_pose_subscriber = n.subscribe("/move_base_simple/goal", 1, goalPoseCallback);

    // Spin and process callbacks
    ros::spin();

    return 0;
}
