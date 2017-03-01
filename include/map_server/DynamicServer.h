#pragma once
#include "MapServer.h"
#include "../common.h"

namespace map_server {

class DynamicServer: public MapServer {
public:
    DynamicServer(ros::NodeHandle nh)
        : nh_(nh),
          cur_scan(NULL),
          timestamp_tolerance_ns_(200000000)

    {
        double probHit, probMiss, thresMin, thresMax;
        nh_.param("sensor_model/hit", probHit, 0.7);
        nh_.param("sensor_model/miss", probMiss, 0.4);
        nh_.param("sensor_model/min", thresMin, 0.12);
        nh_.param("sensor_model/max", thresMax, 0.97);
        nh_.param("map_scale", map_scale_, 50);


        cur_scan = new OcTreeT(m_res);
        cur_scan->setProbHit(probHit);
        cur_scan->setProbMiss(probMiss);
        cur_scan->setClampingThresMin(thresMin);
        cur_scan->setClampingThresMax(thresMax);

        pub_dy   = nh_.advertise<sensor_msgs::PointCloud2>("dy_obj", 1);
        pub_map  = nh_.advertise<sensor_msgs::PointCloud2>("map", 1);
        subCloud = nh_.subscribe(
                    "/dynamicMapping", 1, &DynamicServer::loamCallback, this);

        // initialize octomap object & params
        transform_queue_.clear();
        T_G_B_ = Eigen::Isometry3f::Identity();
        T_B_D_ = Eigen::Isometry3f::Identity();
        T_B_S_ = Eigen::Isometry3f::Identity();
    }

    virtual ~DynamicServer() {}

    void loamCallback        (const doom::LoamScanPtr&     loam);

private:

    void updateMap(const point3d &sensorOrigin);
    void insertPC(const Eigen::Matrix4f &trans, const PointCloud &pc, OcTreeT *tree);
    void checkDiff(void);
    void insertTimeScan(const Eigen::Matrix4f &trans, const doom::LoamScanPtr& loam);

    void lookTF(const doom::LoamScanPtr& loam, Eigen::Matrix4f& sensorToWorld)
    {
        tf::StampedTransform sensorToWorldTf;
        try {
          m_tfListener.lookupTransform(m_worldFrameId, loam->header.frame_id, loam->header.stamp, sensorToWorldTf);
        } catch(tf::TransformException& ex){
          return;
        }
        pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

        return;
    }

    void publishCloud(const ros::Time& rostime, double pose_z)
    {
        sensor_msgs::PointCloud2 dy_out;
        pcl::toROSMsg (dy_pc, dy_out);
        dy_out.header.frame_id = "world";
        dy_out.header.stamp = rostime;
        pub_dy.publish(dy_out);

        // publish Color Points
        PointCloudColor cloud_map;
        for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth),
            end = m_octree->end(); it != end; ++it)
        {
            if (m_octree->isNodeOccupied(*it) &&
                   it.getZ() > (pose_z - 3) &&
                    it.getZ() < (pose_z + 3)){

                PointColor point;
                point.x = it.getX();
                point.y = it.getY();
                point.z = it.getZ();
                double color = (it.getZ()-pose_z + 3)/6.0;

		//point.r = 150 + color*100
		//point.g = 150 + color*100
		//point.b = 150 + color*100

                if (color > 0.66){
                    point.r = 0;
                    point.g = 250;
                    point.b = 0;
                }else if(color > 0.40){
                    point.r = 0;
                    point.g = 0;
                    point.b = 250;
                }else {
                    point.r = 250;
                    point.g = 0;
                    point.b = 0;
		    }
                cloud_map.push_back(point);
            }
        }
        sensor_msgs::PointCloud2 map_out;
        pcl::toROSMsg (cloud_map, map_out);
        map_out.header.frame_id = "world";
        map_out.header.stamp = rostime;
        pub_map.publish(map_out);

    }

    int64_t timestamp_tolerance_ns_;

protected:

    // Ros Sub and Pub
    std::string change_id_frame;
    ros::NodeHandle nh_;
    ros::Subscriber subCloud;
    ros::Publisher  pub_dy;
    ros::Publisher  pub_map;

    // PreOctree
    octomap::OcTree* cur_scan;

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

    // map scale
    int map_scale_;

    // Transform queue, used only when use_tf_transforms is false.
    std::deque<geometry_msgs::TransformStamped> transform_queue_;
};

} /* namespace octomap */

#include "impl/DynamicServer.hpp"
