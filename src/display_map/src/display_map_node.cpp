#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>


int main(int argc, char** argv ){
    ros::init(argc, argv, "pub_displaymap");
    ros::NodeHandle n;

    // map's publisher
    ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);

    // loading map using OpenCV
    std::string imagePath = "src/display_map/src/diag_map.png";
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);

    // error management
    if (image.empty())
    {
        std::cerr << "Failed to load the image from path: " << imagePath << std::endl;
        return -1;
    }

    // map message to be published
    nav_msgs::OccupancyGrid map_msg;

    // map properties - auto detected
    map_msg.info.width = image.cols;
    map_msg.info.height = image.rows;
    map_msg.info.resolution = 0.05; 

    // convert .png image data to occupancy grid data (i.e. .png -> OccupancyGrid msg)
    map_msg.data.resize(map_msg.info.width * map_msg.info.height);
    for (int y = 0; y < image.rows; ++y)
    {
        for (int x = 0; x < image.cols; ++x)
        {
            // TODO: CHECK, DOES NOT WORK PROPERLY
            // [0, 100] = prob. of going in that pixel
            // white pixels=free space
            // black pixels=obstacles
            // transparent pixels=nothing
            // to do so, we need the 4th dimension of image color
            uchar alpha = image.at<cv::Vec4b>(y, x)[3];

            if (alpha == 0)
            {
                // transparent pixel = unknown
                map_msg.data[y * map_msg.info.width + x] = -1;
            }
            else
            {
                // non-transparent pixel
                int value = (image.at<cv::Vec4b>(y, x)[0] > 200) ? 0 : 100;
                map_msg.data[y * map_msg.info.width + x] = value;
            }
        }
    }

    // publish the map
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