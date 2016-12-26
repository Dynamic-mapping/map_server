#pragma once
#include "MapServer.h"
#include "../common.h"

namespace map_server {

class DynamicServer: public MapServer {
public:
    DynamicServer(ros::NodeHandle nh)
        : nh_(nh),
          timestamp_tolerance_ns_(200000000)
    {
        readParameters();
        init();
        subscribe();
        advertise();
    }

    virtual ~DynamicServer() {}

    void laserCallback        (const sensor_msgs::PointCloud2Ptr&     cloud);

private:

    void init();
    void readParameters();
    void subscribe();
    void advertise();

    void updateMap(const Transformation &sCur, const Transformation &sPre);

    int64_t timestamp_tolerance_ns_;

protected:

    // Ros Sub and Pub
    std::string change_id_frame;
    ros::NodeHandle nh_;
    ros::Subscriber subCloud;
    ros::Publisher  pubCenterMap;

    tf::TransformBroadcaster tf_broader_;

    // G is the fixed Ground frame, B is the body frame of the robot, S is the
    // Sensro frame creating the pointclouds, and D is the 'dynamic' frame; i.e.,
    // incoming messages are assumed to be T_G_D.
    Transformation T_B_S_;
    Transformation T_B_D_;
    Transformation T_G_B_;

    // Transform queue, used only when use_tf_transforms is false.
    std::deque<geometry_msgs::TransformStamped> transform_queue_;
};

} /* namespace octomap */

#include "impl/DynamicServer.hpp"
