#pragma once

// SYS LIB
#include <cmath>
#include <vector>
#include <iostream>
#include <string>
#include <math.h>
#include <algorithm>
#include <stdexcept>
#include <functional>

// GOOGLE LIB
#include <glog/logging.h>
#include <gflags/gflags.h>


// EIGEN LIB
#include <eigen3/Eigen/Dense>
#include <Eigen/StdVector>

// CERES LIB
#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/types.h>
#include <ceres/rotation.h>

// ROS LIB
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>

// ROS MESSAGE
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <doom/LaserScan.h>
#include <doom/LoamScan.h>
//#include <volumetric_msgs/SetBoxOccupancy.h>
//#include <volumetric_msgs/SetDisplayBounds.h>

// TF LIB
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// OCOTOMAP LIB
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>

// OPENCV LIB
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

// PCL LIB
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl-1.8/pcl/common/transforms.h>
#include <pcl-1.8/pcl/conversions.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <pcl-1.8/pcl/filters/filter.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <pcl-1.8/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/registration/icp.h>
#include <pcl-1.8/pcl/io/ply_io.h>

#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

const int       IMU_QUE_LEN     = 200;
const int       N_SCANS         = 16;
const double    SCAN_PERIOD     = 0.1;
const int       CLOUD_CACHE     = 10000;
const double    THRES_CURVA     = 0.2;
const int       MAX_ITERATION   = 30;
const float     LM_THRES        = 0.4;
const float     CAR_HEIGHT      = 1.6;
const float     POINT_NORM      = 0.1;

const int       MAP_RADIUS      = 80;

using std::sin;
using std::cos;
using std::atan2;
using std::string;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

enum LABEL {NON, CORNER, CORNER_LESS, SURF, SURF_LESS};


// tfpedef for pcl
typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointColor;
typedef pcl::PointXYZI PointTI;
typedef pcl::PointXY PointPlane;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointTI> PointCloudI;
typedef pcl::PointCloud<PointTI>::Ptr PointCloudIPtr;
typedef pcl::PointCloud<PointTI>::ConstPtr PointCloudIConstPtr;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
typedef pcl::PointCloud<PointColor> PointCloudColor;
typedef pcl::PointCloud<PointColor>::Ptr PointCloudColorPtr;
typedef pcl::PointCloud<int> PointCloudIndex;
typedef pcl::ModelCoefficients::Ptr ModelPtr;
typedef pcl::KdTreeFLANN<PointT> KDtree;
typedef KDtree::Ptr KDtreePtr;
typedef Eigen::Isometry3f Transformation;

struct BASIC {
    float x;
    float y;
    float z;
};

struct TIMER:   public BASIC {};
struct ANGLE:   public BASIC {};
struct ACCEL:   public BASIC {};
struct VELOC:   public BASIC {};
struct SHIFT:   public BASIC {};

struct LINE {
    bool valid;
    PointT start;
    PointT end;
};

struct PLANE {
    bool valid;
    PointT a;
    PointT b;
    PointT c;
};

struct PFeature{

    float   curva;
    float   scanTime;
    int     scanInd;


    int     neiborPick;
    int     label;
    int     scanStart;
    int     scanEnd;

    PointT  point;
};

void PointsToPlane (const PointT &point_a, const PointT &point_b, const PointT &point_c,
                    const ModelPtr plane)
{
    // Create Eigen plane through 3 points
    Eigen::Hyperplane<float, 3> eigen_plane = Eigen::Hyperplane<float, 3>::Through (point_a.getArray3fMap (),
                                                                                    point_b.getArray3fMap (),
                                                                                    point_c.getArray3fMap ());
    plane->values.resize (4);
    for (int i = 0; i < plane->values.size (); i++) plane->values[i] = eigen_plane.coeffs ()[i];
}

inline double rad2deg (double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

inline double Square(const PointT point)
{
    return (pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
}

inline double pointNorm(const PointT point)
{
    return sqrt(Square(point));
}

inline PointT pointAdd(const PointT &p1, const PointT &p2, double pa1, double pa2)
{
    PointT p_out;
    p_out.x = p1.x * pa1 + p2.x * pa2;
    p_out.y = p1.y * pa1 + p2.y * pa2;
    p_out.z = p1.z * pa1 + p2.z * pa2;
    return p_out;
}

inline PointT pointCross2(const PointT &p1, const PointT &p2)
{
    PointT cross;
    cross.x = p1.y * p2.z - p1.z * p2.y;
    cross.y = p1.z * p2.x - p1.x * p2.z;
    cross.z = p1.x * p2.y - p1.y * p2.x;
    return cross;
}

inline PointT pointCross3(PointT p1, PointT p2, PointT p3)
{
    return pointCross2(pointCross2(p1, p2), p3);
}

inline PointT transPoint(PointT pIn, Eigen::MatrixXf rot, Eigen::VectorXf trans)
{
    PointT pOut;
    Eigen::VectorXf point(3);
    point(0) = pIn.x; point(1) = pIn.y; point(2) = pIn.z;
    point = rot*point + trans;
    pOut.x = point(0); pOut.y = point(1); pOut.z = point(2);
    return pOut;
}

Eigen::Isometry3f axisAngleToIso(const float* transform)
{
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    Eigen::Matrix3f rot;
    ceres::AngleAxisToRotationMatrix(transform, rot.data());
    pose.linear() = rot;
    pose.translation() = Eigen::Vector3f(transform[3], transform[4], transform[5]);
    return pose;
}

template <typename T>
ceres::MatrixAdapter<T, 1, 4> ColumnMajorAdapter4x3(T* pointer)
{
    return ceres::MatrixAdapter<T, 1, 4>(pointer);
}


void isoToAngleAxis(const Transformation &pose, float *cam)
{
    ceres::RotationMatrixToAngleAxis(ColumnMajorAdapter4x3(pose.linear().data()), cam);
    Eigen::Vector3f t(pose.translation());
    cam[3] = t.x();
    cam[4] = t.y();
    cam[5] = t.z();
}

void geoToEigen(const geometry_msgs::Transform &trans, Transformation *trans_mat)
{
    geometry_msgs::Quaternion t_q = trans.rotation;
    geometry_msgs::Vector3 t_t = trans.translation;
    Eigen::Matrix3f mat3 = Eigen::Quaternionf(t_q.w, t_q.x, t_q.y, t_q.z).toRotationMatrix();
    Eigen::Matrix4f mat4 = Eigen::Matrix4f::Identity();
    mat4.block(0,0,3,3) = mat3;
    mat4(0,3) = t_t.x;
    mat4(1,3) = t_t.y;
    mat4(2,3) = t_t.z;
    trans_mat->matrix() = mat4;
    return;
}

void eigenToGeo(const Transformation &trans_mat, geometry_msgs::Transform *trans)
{
    Eigen::Quaternionf t_r;
    t_r.matrix() = trans_mat.rotation();
    Eigen::Vector3f t_t = trans_mat.translation();

    trans->translation.x = t_t[0];
    trans->translation.y = t_t[1];
    trans->translation.z = t_t[2];

    trans->rotation.x = t_r.x();
    trans->rotation.y = t_r.y();
    trans->rotation.z = t_r.z();
    trans->rotation.w = t_r.w();
    return;
}

void voxFilter(PointCloudPtr &cloud, double res)
{
    pcl::VoxelGrid<PointT> vox_filter;
    vox_filter.setInputCloud(cloud);
    vox_filter.setLeafSize(res, res, res);
    vox_filter.filter(*cloud);
    return;
}

void groundFilter(const PointCloudPtr &cloud, PointCloud &ground, PointCloud &nonground)
{
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);


    // Set all the parameters for plane fitting
    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.2);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    // Get the points associated with the planar surface
    extract.setNegative(false);
    extract.filter(ground);

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(nonground);
    return;
}
