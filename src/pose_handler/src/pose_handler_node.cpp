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
    double f; // f(n) = h(n) + g(n)
    Node* parent;

    // constructor as seen during lectures
    Node(double x, double y, double f, Node* parent = nullptr) : x(x), y(y), f(f), parent(parent) {}
};

// function to compare between priority queues
struct CompareNodes {
    bool operator()(const Node* a, const Node* b) const {
        return a->f > b->f;
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





/*
    #######################################
    ####    ALGORITHM SIDE
    #######################################
*/

// heuristic function
double h(double curr_x, double curr_y, double goal_x, double goal_y){
    return std::sqrt(std::pow(goal_x - curr_x, 2) + std::pow(goal_y - curr_y, 2));
}

// cost function
double g(){
    return 0.0;
}


// A* algorithm
std::vector<geometry_msgs::PoseStamped> a_star(double start_x, double start_y, double goal_x, double goal_y)
{
    if (!global_map) {
        ROS_ERROR("No map received");
        return std::vector<geometry_msgs::PoseStamped>();
    }

    int start_grid_x, start_grid_y, goal_grid_x, goal_grid_y;

    // converting from World's coordinates to Grid's ones
    worldToGrid(start_x, start_y, start_grid_x, start_grid_y);
    worldToGrid(goal_x, goal_y, goal_grid_x, goal_grid_y);

    ROS_INFO("WORLD %lf, %lf, %lf, %lf", start_x, start_y, goal_x, goal_y);
    ROS_INFO("GRID %d, %d, %d, %d", start_grid_x, start_grid_y, goal_grid_x, goal_grid_y);


    // priority queue allows us to discretize between elements order once push in list
    std::priority_queue<Node*, std::vector<Node*>, CompareNodes> open_set;

    // unordered_set is literally a set where we don't care about the order of nodes
    std::unordered_set<Node*> closed_set;

    // setting up the root of the A* tree
    Node* start_node = new Node(start_grid_x, start_grid_y, 0);
    open_set.push(start_node);

    // iterating while there exists elements in open set
    while (!open_set.empty())
    {
        Node* current = open_set.top();
        open_set.pop();
        ROS_INFO("### CURRENT %f %f, f(n) %f", current->x, current->y, current->f);

        // adding root to the closed set
        closed_set.insert(current);

        // goal found
        if (current->x == goal_grid_x && current->y == goal_grid_y)
        {
            std::vector<geometry_msgs::PoseStamped> path;
            // converting back from grid to world 
            while (current != nullptr)
            {
                geometry_msgs::PoseStamped pose;
                gridToWorld(current->x, current->y, pose.pose.position.x, pose.pose.position.y);
                path.push_back(pose);
                current = current->parent;

            }

            // reversing the path (i.e. going from the start to the end)
            std::reverse(path.begin(), path.end());
            return path;
        }

        // iteration (assuming 8-connected tiles)
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                // if current node
                if (dx == 0 && dy == 0) continue;

                // computing next position to evaluate
                int nx = current->x + dx;
                int ny = current->y + dy;
                ROS_INFO("%d %d %d %d", nx, ny, dx, dy);

                // TODO: check if cell is within map boundaries

                // ROS uses a vector representation of a matrix
                int index = ny * global_map->info.width + nx;

                // if we are in a "walkable" place
                if (global_map->data[index] == 0)
                {
                    double heuristic = h(nx, ny, goal_grid_x, goal_grid_y);
                    double cost = 0; //g();
                    double evaluation_function = heuristic + cost;

                    Node* neighbor = new Node(nx, ny, evaluation_function, current);

                    if (closed_set.find(neighbor) == closed_set.end())
                    {
                        open_set.push(neighbor);
                        closed_set.insert(neighbor);
                    }
                }
            }
        }
    }


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
    std::vector<geometry_msgs::PoseStamped> path = a_star(
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

    // add poses to path path message
    for (const auto& pose : path)
    {
        path_msg.poses.push_back(pose);
    }

    while (ros::ok())
    {
        // I've read that this is usefull
        path_msg.header.stamp = ros::Time::now();

        // publish
        path_publisher.publish(path_msg);
        ros::spinOnce();

        // publish at rate 1 Hz
        ros::Rate(1).sleep();
    }

    return 0;
}
