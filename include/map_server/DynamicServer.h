#pragma once
#include "MapServer.h"
#include "../common.h"

namespace map_server {

class DynamicServer: public MapServer {
public:
    DynamicServer(ros::NodeHandle nh)
        : nh_(nh),
          cur_tree(NULL),
          timestamp_tolerance_ns_(200000000)

    {
        double probHit, probMiss, thresMin, thresMax;
        nh_.param("sensor_model/hit", probHit, 0.7);
        nh_.param("sensor_model/miss", probMiss, 0.4);
        nh_.param("sensor_model/min", thresMin, 0.12);
        nh_.param("sensor_model/max", thresMax, 0.97);

        cur_tree = new OcTreeT(m_res);
        cur_tree->setProbHit(probHit);
        cur_tree->setProbMiss(probMiss);
        cur_tree->setClampingThresMin(thresMin);
        cur_tree->setClampingThresMax(thresMax);

        pub_dy = nh_.advertise<sensor_msgs::PointCloud2>("dy_obj", 1);

        // initialize octomap object & params
        readParameters();
        init();
        subscribe();
    }

    virtual ~DynamicServer() {}

    void loamCallback        (const doom::LoamScanPtr&     loam);

private:

    void init();
    void readParameters();
    void subscribe();

    void updateMap(const point3d &sensorOrigin);
    void insertPC(const Eigen::Matrix4f &trans, const PointCloud &pc, OcTreeT *tree);
    void checkDiff(const Eigen::Matrix4f &trans);
    void insertTimeScan(const Eigen::Matrix4f &trans, const doom::LoamScanPtr& loam);

    int64_t timestamp_tolerance_ns_;

protected:

    // Ros Sub and Pub
    std::string change_id_frame;
    ros::NodeHandle nh_;
    ros::Subscriber subCloud;
    ros::Publisher  pub_dy;

    // PreOctree
    octomap::OcTree* cur_tree;

    // Pointcloud
    PointCloud  dy_pc;
    // tf
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
