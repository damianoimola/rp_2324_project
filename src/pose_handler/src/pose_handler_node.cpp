#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <queue>
#include <list>
#include <unordered_set>
#include <cmath>
#include <limits>
#include <unistd.h>
#include <chrono>

// #include "compute_dmap.h"
#include "rp_loc/dmap_localizer.h"
#include "rp_base/draw_helpers.h"
#include "rp_base/grid_map.h"


#define DEBUG false




bool goal_received = false;
bool init_received = false;
double map_resolution;
std::vector<double> obstacles_distances;

geometry_msgs::Pose global_initial_pose;
geometry_msgs::Pose global_goal_pose;
nav_msgs::OccupancyGrid::ConstPtr global_map;



// a node in the grid
struct Node
{
    double x, y;
    double f, heu, cost; // f(n) = h(n) + g(n)
    Node* parent;

    // constructor as seen during lectures
    Node(double x, double y, double f, double heu, double cost, Node* parent = nullptr) : x(x), y(y), f(f), heu(heu), cost(cost), parent(parent) {}
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
double g(double curr_x, double curr_y){
    int index = curr_x * global_map->info.width + curr_y;

    double element = obstacles_distances[index];
    element = element/255.0;

    return (1.0 - element);
}

// distance map
std::vector<double> compute_distance_map(string filename, float resolution, float dmax) {
  // load the map
  GridMap grid_map(0,0,resolution);
  grid_map.loadFromImage(filename.c_str(), resolution);

  DMapLocalizer localizer;
  localizer.setMap(grid_map, dmax);
  cerr << "rows:  " << localizer.distances.rows << " cols: " << localizer.distances.cols << endl;

  const auto& distances = localizer.distances;
  
  // compute normalization of the DMAP
  float f_min=1e9;
  float f_max=0;
  for(auto& f: distances.cells) {
    f_min=std::min(f, f_min);
    f_max=std::max(f, f_max);
  }
  float scale = 255./(f_max-f_min);

  vector<double> obstacles_distances;
  for (size_t i=0; i<distances.cells.size(); ++i) {
    // obstacles_distances.push_back(scale  * (distances.cells[i] - f_min));
    obstacles_distances.push_back(distances.cells[i]);
  }
  std::reverse(obstacles_distances.begin(), obstacles_distances.end());
  
  ROS_INFO("num of cells %ld, random sample %f", distances.cells.size(), distances.cells[500]);
  
  return obstacles_distances;
}

// A* algorithm
std::vector<geometry_msgs::PoseStamped> a_star(double start_x, double start_y, double goal_x, double goal_y, double alpha, double beta, std::string mode)
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
    Node* start_node = new Node(start_grid_x, start_grid_y, 0, 0, 0);
    open_set.push(start_node);

    // iterating while there exists elements in open set
    while (!open_set.empty())
    {
        Node* current = open_set.top();
        open_set.pop();

        #ifdef DEBUG
        ROS_INFO("### CURRENT %f %f, f(n) %f, g(n) %f, h(n) %f, size %ld",
            current->x, current->y, current->f, current->cost, current->heu, open_set.size());
        #endif

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

                // TODO: check if cell is within map boundaries

                // ROS uses a vector representation of a matrix
                int index = ny * global_map->info.width + nx;

                // if we are in a "walkable" place
                if (global_map->data[index] == 0)
                {
                    double heuristic = mode == "UCS" ? 0 : beta * h(nx, ny, goal_grid_x, goal_grid_y);
                    double cost = mode == "GBFS" ? 0 : alpha * g(nx, ny) + current->cost;
                    // double cost = alpha * g(nx, ny);
                    // double cost = 0;
                    double evaluation_function = heuristic + cost;

                    Node* neighbor = new Node(nx, ny, evaluation_function, heuristic, cost, current);

                    bool found = false;
                    for (const auto& elem: closed_set) {
                        if(elem->x == neighbor->x
                        && elem->y == neighbor->y
                        && elem->f == neighbor->f
                        && elem->cost == neighbor->cost
                        && elem->heu == neighbor->heu){
                            found = true;
                            break;
                        }
                    }

                    // if (closed_set.find(neighbor) == closed_set.end())
                    if(!found)
                    {
                        // ROS_INFO("added %d %d %f %f %f %p", nx, ny, evaluation_function, heuristic, cost, current);
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





/*
    #######################################
    ####    MAIN
    #######################################
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_posehandler");

    if(argc < 5) {ROS_ERROR("USAGE: <alpha> <beta> <dmax> <mode>\nSUGGESTED: 3.0 1.0 0.5 A*"); return -1;}

    double alpha = atof(argv[1]);
    double beta = atof(argv[2]);
    double dmax = atof(argv[3]);
    std::string mode = argv[4];

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

    // obtaining the distance map
    ROS_INFO("alpha %f, dmax %f resolution %f", alpha, dmax, map_resolution);
    ROS_INFO("computing distance map");
    obstacles_distances = compute_distance_map("./diag_map.png", map_resolution, dmax);
    
    // before procees we need to be sure of having poses
    while ((!init_received || !goal_received) && ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("waiting for initial and goal pose");
    }

    // run A* algorithm
    ROS_INFO("### SEARCHING PATH");
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<geometry_msgs::PoseStamped> path = a_star(
        global_initial_pose.position.x,
        global_initial_pose.position.y,
        global_goal_pose.position.x,
        global_goal_pose.position.y,
        alpha, beta, mode
    );
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    ROS_INFO("### PATH FOUND, number of poses: %ld, time elapsed %ld sec.", path.size(), duration.count()/1000000);

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
