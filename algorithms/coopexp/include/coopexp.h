#include "ros/ros.h"
#include <tf/transform_listener.h> // listener
#include <tf/transform_broadcaster.h> // broadcasters
#include <geometry_msgs/PointStamped.h> // PointStamped
#include "nav_msgs/OccupancyGrid.h" // map
#include <math.h> // cos() sen()
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>

// basic file operations
#include <iostream>
#include <fstream>

class coopexp
{
  public:
    coopexp();
    ~coopexp();

  private:
    //params
    std::string robot_name;
    int number_of_robots, robot_number, comunication, real;
    double frequency, start_time;
    double height, length, resolution;
    double robot_radius;
    double inicial_x, inicial_y, inicial_theta;
    double alfa, beta, gama, omega;
    std::string path;

    //atributes
    bool new_frontiers, new_position, errorCostmap, idle, hibernate, multiRobot;

    int nrows, ncols;
    int receiveBufferSize, sendBufferSize;
    int base_link_i, base_link_j;
    int unknownCell, freeCell, frontierCell, inaccessibleCell;
    int index_i_minCost, index_j_minCost;

    double rep,startTime;
    double timeBufferSize;
    double base_link_x, base_link_y;
    double max_cost, max_utility, max_dist;
    double xOrigin, yOrigin;

    std::string frontiers_frame, map_frame, base_link_frame, costmap_frame, utilitymap_frame;
    std::string position_topic, status_topic, markCell_topic, costmap_metadata_topic, utilitymap_metadata_topic;
    std::string path_positions, path_grids;

    ros::NodeHandle n;

    ros::Subscriber frontiers_sub, status_sub, position_sub;
    ros::Publisher status_pub, markCell_pub, costmap_pub, costmap_metadata_pub, utilitymap_pub, utilitymap_metadata_pub;

    tf::TransformListener *listener;
    tf::TransformBroadcaster map_to_costmap_broadcaster, map_to_utilitymap_broadcaster;
    tf::StampedTransform map_to_costmap_transform, map_to_utilitymap_transform;

    nav_msgs::OccupancyGrid frontiers, costmapOccupancyGrid, utilitymapOccupancyGrid;
    std_msgs::Float64MultiArray costmap, utilitymap, robotsPosition;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac;
    move_base_msgs::MoveBaseGoal goal; 

    //methods
    bool ready(void);
    void start(void);

    void updateTf(void);

    int index(int, int);
    int noFrontiers(int, int);

    double minDistOtherRobots(int, int);
    double minCost(int, int);
    
    void frontiersReceived(const nav_msgs::OccupancyGrid::ConstPtr& frontiers_ptr);
    void positionReceived(const std_msgs::Float64MultiArray::ConstPtr& msg_ptr);
    void statusReceived(const std_msgs::Float64MultiArray::ConstPtr& msg_ptr);

    void updateRobotPosition(void);
    void updatePositions(void);
    
    void initCostmap(void);
    void updateCostmap(int,int,int);
    void updateUtilitymap(void);
    void publishCostmap(void);
    
    void generateGoal(void);
    void sendGoal(double, double, double);
    void sendStatus(double, double, double, double);
    void markCell(int,int,int);
    void markFrontiers(void);

    bool readPos(int);

    double getX(int i);
    double getY(int j);
    int getI(double x);
    int getJ(double y);
};