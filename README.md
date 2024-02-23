# Simple Planner | Robot Programming 2023/2024
Instruction on use will follow.

## Briefly
This project is intended to build a simple planning system using ROS and RVIZ.

The main node that handles everything is the **pose_handler_node**. Recalling that an evaluation function f(n) is the sum of a cost function g(n) and a heuristic function h(n), there are 4 parameters that this node will get to be correctly launched:
-    **Alpha** controls the weight given to the cost function
-    **Beta** controls the weight of the heuristic function
-    **Dmax** is a parameter to handle values of the distance map
-    **Mode** let you select between 3 algorithm to use: **A\*** (i.e. `f(n)=g(n)+h(n)`), **UCS** (Uniform Cost Search, i.e. `f(n)=g(n)`) and **GBFS** (Greedy Best First Search, i.e. `f(n)=h(n)`), in particular, selecting alpha and beta different from 1.0, you will choose the weighted version of each of these algorithm.

##  Execution
Once you are ***in the project folder***, you need 4 different terminals windows to run the program, and the commands needed to be executed are:
1. ```roscore``` essential to allows communication between nodes
2. ```rosrun rviz rviz``` to execute RVIZ
3. ```rosrun map_server map_server src/display_map/src/diag_map.png  0.05``` to use the pre_built map_server node that allows you to send a map over the topic "/map" to RVIZ specifing the image of the map and its resolution
4. ```
    catkin build
    source devel/setup.bash
    rosrun pose_handler pose_handler_node 3.0 1.0 0.5 A* -DDEBUG
    ```
   in order to:
   -    build the project
   -    source the setup.bash script
   -    execute the node with the following parameters: alpha, beta, dmax, mode.

Once you run the last terminal, you need to setup RVIZ in order to subscribe to published topics, in particular, you need a MAP component that subscribes to topic */map* and a PATH component that subscribes to a topic */path*. After that, you can choose via UI buttons of RVIZ the initial guess and the final pose and let the algorithm do the job. Once the algorithm completes the search, a path will be displayed on RVIZ.

## Example of execution
Using
-    Alpha = 3.0
-    Beta = 1.0
-    Dmax = 0.5

the following are the execution using every mode available.

With UCS (i.e. only cost function `f(n)=g(n)`)
![A* Runs on a DIAG map](/runs/ucs.png)

With GBFS (i.e. only heuristic function `f(n)=h(n)`)
![A* Runs on a DIAG map](/runs/gbfs.png)

With A* (i.e. summing heuristic and cost function `f(n)=g(n)+h(n)`)
![A* Runs on a DIAG map](/runs/astar.png)
