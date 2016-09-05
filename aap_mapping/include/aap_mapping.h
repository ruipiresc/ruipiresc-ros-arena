#include "ros/ros.h"
#include <tf/transform_listener.h> // listener
#include <tf/transform_broadcaster.h> // broadcasters
#include <geometry_msgs/PointStamped.h> // PointStamped
#include "nav_msgs/OccupancyGrid.h" // map
#include "nav_msgs/MapMetaData.h" // map_metadata
#include "sensor_msgs/LaserScan.h" // laser
#include <geometry_msgs/Twist.h>
#include <math.h> // cos() sen()
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>

// basic file operations
#include <iostream>
#include <fstream>

#define M_PI 3.14159265358979323846

class aap_mapping
{
  public:
    aap_mapping();
    ~aap_mapping();

  private:
    //params
    std::string robot_name;
    int number_of_robots, robot_number, comunication, real;
    double frequency, start_time;
    double height, length, resolution;
    double robot_radius;
    double inicial_x, inicial_y, inicial_theta;
    std::string path;

    //atributes
    bool newMap, idle, close_all, multiRobot;

    int nrows, ncols;
    int map_buffer, laser_buffer;
    int obstacle_extender;
    int minOccupancy, maxOccupancy, unknownOccupancy;

    double listener_buffer;
    double max_laser_dist, min_laser_dist;
    double occupiedCellProb, emptyCellProb;
    double angularSpeed;
    double xOrigin, yOrigin;

    std::string global_map_frame, map_frame, odom_frame, laser_frame, base_link_frame;
    std::string position_topic, laser_topic, map_metadata_topic, markCell_topic, cmd_vel_topic, status_topic;
    std::string path_positions, path_grids;
    std::string flag_map, flag_global_map;
    std::string global_grid_name, grid_name;

    ros::NodeHandle n;

    ros::Publisher map_pub, map_metadata_pub, position_pub;
    ros::Subscriber status_sub, laser_sub, global_map_sub, markCell_sub, cmd_vel_sub, position_sub;

    tf::TransformListener *listener_base_laser_link;
    tf::TransformListener *listener_base_link;
    tf::TransformBroadcaster map_to_odom_broadcaster, global_map_to_map_broadcaster;
    tf::StampedTransform map_to_odom_transform, global_map_to_map_transform;

    std_msgs::Float64MultiArray robotsPosition;

    nav_msgs::OccupancyGrid map;

    //methods
    int index(int, int);
    int closestRobot(int, int);

    void inicializer(void);
    void startSubscribing(void);

    void scanReceived(const sensor_msgs::LaserScan::ConstPtr& data_ptr);
    void globalMapReceived(const nav_msgs::OccupancyGrid::ConstPtr& global_map_ptr);
    void velReceived(const geometry_msgs::Twist& cmd_vel_ptr);
    void positionReceived(const std_msgs::Float64MultiArray::ConstPtr& msg_ptr);
    void statusReceived(const std_msgs::Float64MultiArray::ConstPtr& msg_ptr);

    void newData(double, double, bool);
    void changeData(int, int, double);
    void inflateObstacles(int, int, double);
    void checkGlobalMap(void);
    
    void updateRobotPosition(void);
    void updatePositions(void);

    void publishPosition(void);
    void publishMap(void);

    double getX(int i);
    double getY(int j);
    int getI(double x);
    int getJ(double y);
};