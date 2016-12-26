#include <ros/ros.h>
#include "map_server/MapServer.h"


using namespace map_server;

int main(int argc, char** argv){
  ros::init(argc, argv, "map_server");
  const ros::NodeHandle& private_nh = ros::NodeHandle("~");

  MapServer server;
  ros::spin();

  return 0;
}
