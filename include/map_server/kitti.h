#pragma once
#include "MapServer.h"
#include "../common.h"

namespace map_server{

class KittiServer: public MapServer {
public:
    KittiServer(ros::NodeHandle nh)
        :nh_(nh)
    {
        readParameters();
        init();
        subscribe();
        advertise();
    }

    virtual ~KittiServer() {}

    void cloudCB   (const sensor_msgs::PointCloud2Ptr& pc);

private:

    void init();
    void readParameters();
    void subscribe();
    void advertise();

    void updateMap(const point3d &sensorOrigin);
    void insertPC(const Eigen::Matrix4f &trans, const PCLPointCloud& pc);


protected:

    // ROS sub and pub
    std::string id_frame_;
    ros::NodeHandle nh_;
    ros::Subscriber subCloud_;
    ros::Publisher  pubMap_;

    tf::TransformBroadcaster tf_broader_;

    double vox_re_;
    // G is the fixed Ground frame, B is the body frame of the robot, S is the
    // Sensro frame creating the pointclouds, and D is the 'dynamic' frame; i.e.,
    // incoming messages are assumed to be T_G_D.
    Transformation T_B_S_;
    Transformation T_B_D_;
    Transformation T_G_B_;

    // Transform queue, used only when use_tf_transforms is false.
    std::deque<geometry_msgs::TransformStamped> transform_queue_;
};
} /*namespace map_server*/

#include "impl/kitti.hpp"
