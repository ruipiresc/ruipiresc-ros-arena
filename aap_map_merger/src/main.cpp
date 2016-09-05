#include "ros/ros.h"
#include "aap_map_merger.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aap_map_merger");
  aap_map_merger my_class;
  return(0);
}
