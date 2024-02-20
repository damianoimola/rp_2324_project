#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>


int main(int argc, char** argv ){
    ros::init(argc, argv, "displaymap");
    ros::NodeHandle n;

    // map's publisher
    ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);

    // map message to be published
    nav_msgs::OccupancyGrid map_msg;

    // map's info (found online), to be filled (width, height, resolution, etc.)
    map_msg.info.width = 2138;
    map_msg.info.height = 987;
    map_msg.info.resolution = 0.5;

    // TODO: i have to set the map data

    // Publish the map
    while (ros::ok())
    {
        // i've read that this is usefull
        map_msg.header.stamp = ros::Time::now();

        // publish
        map_publisher.publish(map_msg);
        ros::spinOnce();

        // publish at rate 1 Hz
        ros::Rate(1).sleep();
    }

    return 0;
}