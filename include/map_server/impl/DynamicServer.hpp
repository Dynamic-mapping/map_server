#pragma once
#include "map_server/DynamicServer.h"
#include "common.h"

using namespace map_server;

void DynamicServer::init()
{
    transform_queue_.clear();
    T_G_B_ = Eigen::Isometry3f::Identity();
    T_B_D_ = Eigen::Isometry3f::Identity();
    T_B_S_ = Eigen::Isometry3f::Identity();
}

void DynamicServer::readParameters()
{

}

void DynamicServer::subscribe()
{
    subCloud = nh_.subscribe(
                "/laserCloud", 1, &DynamicServer::laserCallback, this);
}

void DynamicServer::advertise()
{
    pubCenterMap = nh_.advertise<sensor_msgs::PointCloud2>(
                "centerMap", 1);
}

void DynamicServer::updateMap(const point3d &sensorOrigin)
{
    /// Transform Checking
    if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
      || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
    {
      ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
    }

    /// Check the Old Octree
    OcTreeT oldTree = *m_octree;
    m_octree->clear();
    for (OcTree::leaf_iterator it = oldTree.begin(); it != oldTree.end(); it++) {

        if(oldTree.isNodeOccupied(*it)){

          if (sqrt(pow(it.getX() - sensorOrigin.x(),2) + pow(it.getY() - sensorOrigin.y(),2)) >= 80)
              continue;
          point3d old_point(it.getX(), it.getY(), it.getZ());

          m_octree->updateNode(old_point, true);
        }
    }
}

void DynamicServer::laserCallback(const sensor_msgs::PointCloud2Ptr &cloud)
{

    /// Step1 Obtain the Pointcloud
    ros::WallTime startTime = ros::WallTime::now();
    PCLPointCloud pc; // input cloud for filtering and ground-detection
    pcl::fromROSMsg(*cloud, pc);

    /// Step2 Extract the transformation from tf
    tf::StampedTransform sensorToWorldTf;
    try {
      m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
    } catch(tf::TransformException& ex){
//      ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
      return;
    }
    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    /// Step3 If the pose has gone far enough, reset the map
    static Eigen::Matrix4f previousLong = Eigen::MatrixXf::Zero(4, 4);
    double distance = sqrt(pow(sensorToWorld(0,3) - previousLong(0,3),2) +
                           pow(sensorToWorld(1,3) - previousLong(1,3),2));
    if (distance > 1) {
        point3d sensor_org (sensorToWorld(0,3), sensorToWorld(1,3), sensorToWorld(2,3));
        updateMap(sensor_org);
        previousLong = sensorToWorld;
    }

    /// Step4 set up filter for height range, also removes NANs:
    pcl::PassThrough<PCLPoint> pass_x;
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
    pcl::PassThrough<PCLPoint> pass_y;
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
    pcl::PassThrough<PCLPoint> pass_z;
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

    PCLPointCloud pc_ground; // segmented ground plane
    PCLPointCloud pc_nonground; // everything else

    if (m_filterGroundPlane){
      pass_x.setInputCloud(pc.makeShared());
      pass_x.filter(pc);
      pass_y.setInputCloud(pc.makeShared());
      pass_y.filter(pc);
      pass_z.setInputCloud(pc.makeShared());
      pass_z.filter(pc);
      filterGroundPlane(pc, pc_ground, pc_nonground);

      // transform clouds to world frame for insertion
      pcl::transformPointCloud(pc_ground, pc_ground, sensorToWorld);
      pcl::transformPointCloud(pc_nonground, pc_nonground, sensorToWorld);
    } else {
      // directly transform to map frame:
      pcl::transformPointCloud(pc, pc, sensorToWorld);

      // just filter height range:
      pass_x.setInputCloud(pc.makeShared());
      pass_x.filter(pc);
      pass_y.setInputCloud(pc.makeShared());
      pass_y.filter(pc);
      pass_z.setInputCloud(pc.makeShared());
      pass_z.filter(pc);

      pc_nonground = pc;
      // pc_nonground is empty without ground segmentation
      pc_ground.header = pc.header;
      pc_nonground.header = pc.header;
    }

    /// Step5 for the new Laser Scan, make the ICP alignment

    /// Step6 Insert the pointcloud
    insertScan(sensorToWorldTf.getOrigin(), pc_ground, pc_nonground);
    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_DEBUG("Pointcloud insertion in MapServer done (%zu+%zu pts (ground/nonground), %f sec)", pc_ground.size(), pc_nonground.size(), total_elapsed);

    publishAll(cloud->header.stamp);
}
