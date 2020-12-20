// https://github.com/osrf/sdformat/blob/sdformat6_6.2.0/examples/dom.cc
#include <ros/ros.h>

#include "tailsitter_spawner.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tailsitter_spawner");
  ros::NodeHandle nh("");
  TailsitterSpawner tailsitter_spawner(nh);
  tailsitter_spawner.spawnTailsitter();
  ros::spinOnce();
  return 0;
}