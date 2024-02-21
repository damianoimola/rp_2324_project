#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <limits>
#include <unistd.h>

bool goal_received = false;
bool init_received = false;
double map_resolution;

geometry_msgs::Pose global_initial_pose;
geometry_msgs::Pose global_goal_pose;
nav_msgs::OccupancyGrid::ConstPtr global_map;



// a node in the grid
struct Node
{
    double x, y;
    double cost;
    Node* parent;

    // constructor as seen during lectures
    Node(double x, double y, double cost, Node* parent = nullptr) : x(x), y(y), cost(cost), parent(parent) {}
};

// function to compare between priority queues
struct CompareNodes {
    bool operator()(const Node* a, const Node* b) const {
        return a->cost > b->cost;
    }
};





/*
    #######################################
    ####    CALLBACKS
    #######################################
*/



void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // Process the initial pose information
    geometry_msgs::Pose pose = msg->pose.pose;
    ROS_INFO("Received initial pose: x=%f, y=%f, theta=%f", pose.position.x, pose.position.y, tf::getYaw(pose.orientation));
    init_received=true;
    global_initial_pose = pose;
}

void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Process the goal pose information
    geometry_msgs::Pose pose = msg->pose;
    ROS_INFO("Received goal pose: x=%f, y=%f, theta=%f", pose.position.x, pose.position.y, tf::getYaw(pose.orientation));
    goal_received=true;
    global_goal_pose = pose;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    global_map = map;
    map_resolution = map->info.resolution;
    ROS_INFO("Received Map from map_server, resolution: %lf", map_resolution);
}



/*
    #######################################
    ####    HANDLING VALUES
    #######################################
*/

// Convert world coordinates to grid coordinates
void worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y)
{
    grid_x = (world_x - global_map->info.origin.position.x) / map_resolution;
    grid_y = (world_y - global_map->info.origin.position.y) / map_resolution;
}

// Convert grid coordinates to world coordinates
void gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y)
{
    world_x = global_map->info.origin.position.x + grid_x * map_resolution;
    world_y = global_map->info.origin.position.y + grid_y * map_resolution;
}



// heuristic function
double h(){

}

// cost function
double g(){

}


// Dijkstra's algorithm considering cost based on the distance to the closest obstacle
std::vector<geometry_msgs::PoseStamped> findPath(double start_x, double start_y, double goal_x, double goal_y)
{
    if (!global_map)
    {
        ROS_ERROR("No map received");
        return std::vector<geometry_msgs::PoseStamped>();
    }

    int start_grid_x, start_grid_y, goal_grid_x, goal_grid_y;

    worldToGrid(start_x, start_y, start_grid_x, start_grid_y);
    worldToGrid(goal_x, goal_y, goal_grid_x, goal_grid_y);

    ROS_INFO("WORLD %lf, %lf, %lf, %lf", start_x, start_y, goal_x, goal_y);
    ROS_INFO("GRID %d, %d, %d, %d", start_grid_x, start_grid_y, goal_grid_x, goal_grid_y);



    // No path found
    return std::vector<geometry_msgs::PoseStamped>();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_posehandler");

    // declaring all necessary handlers
    ros::NodeHandle ip_n;
    ros::NodeHandle gp_n;
    ros::NodeHandle map_n;

    // subscription to the /initialpose topic
    ros::Subscriber initial_pose_subscriber = ip_n.subscribe("/initialpose", 1, initialPoseCallback);
    // subscription to the /move_base_simple/goal topic
    ros::Subscriber goal_pose_subscriber = gp_n.subscribe("/move_base_simple/goal", 1, goalPoseCallback);
    // subscription to the /map topic
    ros::Subscriber map_subscriber = map_n.subscribe("/map", 1, mapCallback);



    // before procees we need to be sure of having map
    while (!global_map && ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("waiting for map");
    }
    

    // before procees we need to be sure of having poses
    while ((!init_received || !goal_received) && ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("waiting for initial and goal pose");
    }

    // run A* algorithm
    ROS_INFO("### SEARCHING PATH");
    std::vector<geometry_msgs::PoseStamped> path = findPath(
        global_initial_pose.position.x,
        global_initial_pose.position.y,
        global_goal_pose.position.x,
        global_goal_pose.position.y
    );
    ROS_INFO("### PATH FOUND, number of poses: %ld", path.size());

    // publish the path
    ros::NodeHandle path_n;

    ros::Publisher path_publisher = path_n.advertise<nav_msgs::Path>("/path", 1);
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();

    for (const auto& pose : path)
    {
        path_msg.poses.push_back(pose);
    }

    path_publisher.publish(path_msg);
    ROS_INFO("### PATH PUBLISHED");

    ros::spin();

    return 0;
}
