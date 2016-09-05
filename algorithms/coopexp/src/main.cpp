#include "ros/ros.h"
#include "coopexp.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coopexp");
  coopexp my_class;
  return(0);
}
