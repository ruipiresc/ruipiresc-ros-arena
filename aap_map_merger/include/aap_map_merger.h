#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h" // map
#include "nav_msgs/MapMetaData.h" // map_metadata
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>

// basic file operations
#include <iostream>
#include <fstream>

#define MAXNROBOTS 64

class aap_map_merger
{
  public:
    aap_map_merger();
    ~aap_map_merger();

  private:
    //params
    int number_of_robots, comunication, real;
    double frequency, merge_frequency;
    double height, length, resolution;
    double robot_radius;
    std::string path;

    //atributes
    bool newMap, idle, close_all;

    int nrows, ncols, iterations;
    int map_buffer, positions_buffer;
    int minOccupancy, maxOccupancy, unknownOccupancy;

    double xOrigin, yOrigin;
    
    std::string global_map_frame, robots_map_frames[MAXNROBOTS];
    std::string global_map_metadata_topic, position_topic, status_topic;
    std::string path_positions, path_grids;

    ros::NodeHandle n;

    ros::Publisher global_map_pub, global_map_metadata_pub;

    ros::Subscriber laser_sub, map_sub[MAXNROBOTS], status_sub, position_sub;

    nav_msgs::OccupancyGrid global_map;

    std_msgs::Float64MultiArray robotsPosition;

    //methods
    int index(int, int);
    int closestRobot(int, int);

    void inicializer(void);
    void startSubscribing(void);

    void mapReceived(const nav_msgs::OccupancyGrid::ConstPtr& map_ptr);
    void positionReceived(const std_msgs::Float64MultiArray::ConstPtr& msg_ptr);
    void statusReceived(const std_msgs::Float64MultiArray::ConstPtr& msg_ptr);

    void updatePositions(void);
    
    void publishMap(void);

    void askForMaps(void);
    void updateGlobalMap(void);

    bool readPos(int);
    bool readMap(int);
};