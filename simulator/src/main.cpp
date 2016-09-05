#include "ros/ros.h"
#include "simulator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulator");
  Simulator my_class;
  return(0);
}
