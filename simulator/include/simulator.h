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

class Simulator
{
  public:
    Simulator();
    ~Simulator();

  private:
  	//params
    double iterations, frequency;
    std::string path;

  	//atributes
    std_msgs::Float64MultiArray log, status_in, status_out;
    bool new_status;
    double code,rep,time,ratio;
    double iteration;
    double med, alfa;

    ros::Subscriber status_sub;
    ros::Publisher status_pub;

    std::string status_topic;

    ros::NodeHandle n;

    std::ofstream myfile;

    //methods
    void sendStatus(double, double, double, double);
    void logData(void);
	void statusReceived(const std_msgs::Float64MultiArray::ConstPtr& msg_ptr);
};
	