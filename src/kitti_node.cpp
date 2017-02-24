#include <ros/ros.h>
#include "map_server/kitti.h"


using namespace map_server;

int main(int argc, char** argv){
    ros::init(argc, argv, "map_server");
    ros::NodeHandle nh("~");

    KittiServer server(nh);
    ros::spin();

    return 0;
}
