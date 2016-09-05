#include "ros/ros.h"
#include "aap_mapping.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aap_mapping");
  aap_mapping my_class;
  return(0);
}
