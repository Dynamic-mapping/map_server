#pragma once
#include "MapServer.h"
#include "../common.h"

namespace map_server {

class DynamicServer: public MapServer {
public:
    DynamicServer(ros::NodeHandle nh)
        : nh_(nh),
          imgNode_(nh),
          obj_scan(NULL),
          plane_octree(NULL),
          nonplane_octree(NULL),
          timestamp_tolerance_ns_(200000000)
    {
        double probHit, probMiss, thresMin, thresMax;
        nh_.param("sensor_model/hit", probHit, 0.7);
        nh_.param("sensor_model/miss", probMiss, 0.4);
        nh_.param("sensor_model/min", thresMin, 0.12);
        nh_.param("sensor_model/max", thresMax, 0.97);
        nh_.param("map_scale", map_scale_, 30);


        obj_scan = new OcTreeT(m_res);
        obj_scan->setProbHit(probHit);
        obj_scan->setProbMiss(probMiss);
        obj_scan->setClampingThresMin(thresMin);
        obj_scan->setClampingThresMax(thresMax);

        plane_octree = new OcTreeT(m_res);
        plane_octree->setProbHit(probHit);
        plane_octree->setProbMiss(probMiss);
        plane_octree->setClampingThresMin(thresMin);
        plane_octree->setClampingThresMax(thresMax);

        nonplane_octree = new OcTreeT(m_res);
        nonplane_octree->setProbHit(probHit);
        nonplane_octree->setProbMiss(probMiss);
        nonplane_octree->setClampingThresMin(thresMin);
        nonplane_octree->setClampingThresMax(thresMax);


        pub_octree_plane = nh_.advertise<sensor_msgs::PointCloud2>("octree_plane", 1);
        pub_octree_nonplane = nh_.advertise<sensor_msgs::PointCloud2>("octree_nonplane", 1);

        pub_dy   = nh_.advertise<sensor_msgs::PointCloud2>("dy_obj", 1);
        pub_map  = nh_.advertise<sensor_msgs::PointCloud2>("map", 1);
        pub_ground     = nh_.advertise<sensor_msgs::PointCloud2>("ground", 1);
        pub_nonground  = nh_.advertise<sensor_msgs::PointCloud2>("nonground", 1);
        pub_img  = imgNode_.advertise("map_img", 1);
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
    void insertGround(const Eigen::Matrix4f &trans, const PointCloud &pc);
    void insertNonGround(const Eigen::Matrix4f &trans, const PointCloud &pc);
    void checkDiff(void);
    void insertTimeScan(const Eigen::Matrix4f &trans, const doom::LoamScanPtr& loam);
    void ApproxNearestNeighbors(const PointCloudPtr& points, PointCloud& neighbors);
    void MeasurementUpdate(const PointCloudPtr& points,
                           const PointCloudPtr& neighbors,
                                 PointCloud &aligned);
    void PointCloudCut(const PointCloudPtr& points,
                                      PointCloud& outputs);

    void lookTF(const doom::LoamScanPtr& loam, Eigen::Matrix4f& sensorToWorld, double &yaw)
    {
        tf::StampedTransform sensorToWorldTf;
        try {
          m_tfListener.lookupTransform(m_worldFrameId, loam->header.frame_id, loam->header.stamp, sensorToWorldTf);
        } catch(tf::TransformException& ex){
          return;
        }
        pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
        tf::Quaternion q = sensorToWorldTf.getRotation();
        double r, p, y;
        tf::Matrix3x3(q).getRPY(r, p, y);
        yaw = y;
        return;
    }

    void publishCloud(const ros::Time& rostime, Eigen::Matrix4f pose, double yaw)
    {
        double pose_x = pose(0,3);
        double pose_y = pose(1,3);
        double pose_z = pose(2,3);

        Eigen::Matrix3f quan;
        quan(0,0) = pose(0,0);
        quan(0,1) = pose(0,1);
        quan(0,2) = pose(0,2);
        quan(1,0) = pose(1,0);
        quan(1,1) = pose(1,1);
        quan(1,2) = pose(1,2);
        quan(2,0) = pose(2,0);
        quan(2,1) = pose(2,1);
        quan(2,2) = pose(2,2);

//        Eigen::Quaternionf q(quan);
//        Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
//        float yaw = -euler[0];
        yaw = -yaw;
        std::cout << yaw << std::endl;

        sensor_msgs::PointCloud2 dy_out;
        pcl::toROSMsg (dy_pc, dy_out);
        dy_out.header.frame_id = "world";
        dy_out.header.stamp = rostime;
        pub_dy.publish(dy_out);

        // Add random rotation and translation
        float Tt = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float Tr = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

        float T_e = 0.0 * Tt;
        float R_e = 0.0 * (Tr-0.5);

        int radius = map_scale_/m_res;
        cv::Mat img = cv::Mat::zeros(cv::Size(2*radius+1, 2*radius+1), CV_8UC1);
        cv::Mat e_map (2*radius+1, 2*radius+1, CV_32F, cv::Scalar::all(0));
        PointCloudColor cloud_map;
        for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth),
            end = m_octree->end(); it != end; ++it)
        {
            if (m_octree->isNodeOccupied(*it) &&
                   it.getZ() > (pose_z - 3) &&
                    it.getZ() < (pose_z + 3)){

                PointColor point;
                point.x = it.getX() + Tr * T_e;
                point.y = it.getY() + Tr * T_e;
                point.z = it.getZ();
                double height = (it.getZ()-pose_z+3)/6.0;

                point.r = 50 + 100*height;
                point.g = 50 + 100*height;
                point.b = 50 + 100*height;

                cloud_map.push_back(point);

                int p_x = -((point.y-pose_y)/m_res*cos(R_e)+(point.x-pose_x)/m_res*sin(R_e))
                        + img.cols/2;
                int p_y = -((point.x-pose_x)/m_res*cos(R_e)-(point.y-pose_y)/m_res*sin(R_e))
                        + img.rows/2;

                //int p_x = -(point.y-pose_y)/m_res + img.cols/2;
                //int p_y = -(point.x-pose_x)/m_res + img.rows/2;

                if ( p_x >= 0 && p_x < 2*radius+1 && p_y >=0 && p_y < 2*radius+1){
                  e_map.at<float>(p_x, p_y) = (e_map.at<float>(p_x, p_y)+ height)/2;
                }
            }
        }

        // Cloud plane
        PointCloud cloud_plane;
        cloud_plane.clear();
        for (OcTreeT::iterator it = plane_octree->begin(m_maxTreeDepth),
            end = plane_octree->end(); it != end; ++it)
        {
            if (plane_octree->isNodeOccupied(*it)) {

                PointT point;
                point.x = it.getX();
                point.y = it.getY();
                point.z = it.getZ();

                cloud_plane.push_back(point);
            }
        }

        // publish Color Points
        sensor_msgs::PointCloud2 msg_plane;
        pcl::toROSMsg (cloud_plane, msg_plane);
        msg_plane.header.frame_id = "world";
        msg_plane.header.stamp = rostime;
        pub_octree_plane.publish(msg_plane);


        // Cloud nonPlane
        PointCloud cloud_nonplane;
        cloud_nonplane.clear();
        for (OcTreeT::iterator it = nonplane_octree->begin(m_maxTreeDepth),
            end = nonplane_octree->end(); it != end; ++it)
        {
            if (nonplane_octree->isNodeOccupied(*it)) {

                PointT point;
                point.x = it.getX();
                point.y = it.getY();
                point.z = it.getZ();

                cloud_nonplane.push_back(point);
            }
        }

        // publish Color Points
        sensor_msgs::PointCloud2 msg_nonplane;
        pcl::toROSMsg (cloud_nonplane, msg_nonplane);
        msg_nonplane.header.frame_id = "world";
        msg_nonplane.header.stamp = rostime;
        pub_octree_nonplane.publish(msg_nonplane);



        // publish image
        for (size_t i = 0; i < 2*radius+1; i++)
        {
            for (size_t j = 0; j < 2*radius+1; j++){
                //if (img.at<char>(p_x, p_y) < c_v)
                //  img.at<char>(p_x, p_y) = c_v;
                float height = e_map.at<float>(i, j)*300 + 50;

                if (height >= 240){
                    img.at<char>(i, j)=245;
                    continue;
                }

                if (height <= 10) {
                    img.at<char>(i, j)=10;
                    continue;
                }

                img.at<char>(i, j) = height;
            }
        }


        // publish Color Points
        sensor_msgs::PointCloud2 map_out;
        pcl::toROSMsg (cloud_map, map_out);
        map_out.header.frame_id = "world";
        map_out.header.stamp = rostime;
        pub_map.publish(map_out);


        // Publish image
        cv::resize(img,img,cv::Size(600,600));
        sensor_msgs::ImagePtr im_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
        pub_img.publish(im_msg);
    }

    int64_t timestamp_tolerance_ns_;

protected:

    // Ros Sub and Pub
    std::string change_id_frame;
    ros::NodeHandle nh_;
    ros::Subscriber subCloud;
    ros::Publisher  pub_dy;
    ros::Publisher  pub_map;
    ros::Publisher  pub_ground;
    ros::Publisher  pub_nonground;

    ros::Publisher  pub_octree_plane;
    ros::Publisher  pub_octree_nonplane;

    // Image transport
    image_transport::ImageTransport imgNode_;
    image_transport::Publisher pub_img;

    // PreOctree
    octomap::OcTree* obj_scan;

    octomap::OcTree* plane_octree;
    octomap::OcTree* nonplane_octree;

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
