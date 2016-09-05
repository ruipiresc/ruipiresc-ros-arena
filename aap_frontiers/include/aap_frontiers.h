#include "ros/ros.h"
#include <tf/transform_listener.h> // listener
#include <tf/transform_broadcaster.h> // broadcasters
#include <geometry_msgs/PointStamped.h> // PointStamped
#include "nav_msgs/OccupancyGrid.h" // map
#include "nav_msgs/MapMetaData.h" // map_metadata
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

class aap_frontiers
{
  public:
    aap_frontiers();
    ~aap_frontiers();

  private:
    //params
    std::string robot_name;
    int number_of_robots, robot_number, comunication, real;
    double frequency, start_time;
    double height, length, resolution;
    double robot_radius;
    double inicial_x, inicial_y, inicial_theta;

    //atributes
    bool idle, close_all, multiRobot;

    int nrows, ncols;
    int receiveBufferSize, sendBufferSize;
    int maxOccupancy;
    int unknownCell, freeCell, occupiedCell, frontierCell, inaccessibleCell;
    
    double updateFrontiersPeriod;
    double timeBufferSize;
    double xOrigin, yOrigin;

    std::string frontiers_frame, map_frame, base_link_frame;
    std::string frontiers_metadata_topic, status_topic, markCell_topic;

    ros::NodeHandle n;
    ros::Publisher frontiers_pub, frontiers_metadata_pub;
    ros::Subscriber map_sub, status_sub, markCell_sub;

    tf::TransformListener *listener;
    tf::TransformBroadcaster map_to_frontiers_broadcaster;
    tf::StampedTransform map_to_frontiers_transform;

    nav_msgs::OccupancyGrid frontiers, map;

    //methods
    bool ready(void);
    void start(void);

    void updateTf(void);
    void mapReceived(const nav_msgs::OccupancyGrid::ConstPtr& map_ptr);
    void updateFrontiers(void);
    bool isFrontier(int, int);
    void inflameObstacles(int,int);

    void markCellReceived(const std_msgs::Int32MultiArray::ConstPtr& msg_ptr);
    void statusReceived(const std_msgs::Float64MultiArray::ConstPtr& msg_ptr);

    int index(int, int);
    int convertIndex(int, int, double, double, double);
    double getX(int);
    double getY(int);
};