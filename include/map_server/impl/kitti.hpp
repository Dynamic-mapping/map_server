#pragma once
#include "map_server/kitti.h"
#include "common.h"

using namespace map_server;

void KittiServer::init()
{
    transform_queue_.clear();
    T_G_B_ = Eigen::Isometry3f::Identity();
    T_B_D_ = Eigen::Isometry3f::Identity();
    T_B_S_ = Eigen::Isometry3f::Identity();
}

void KittiServer::readParameters()
{
    nh_.param("vox", vox_re_, double(0.3));
}

void KittiServer::subscribe()
{
    subCloud_ = nh_.subscribe(
                "/kitti_player/hdl64e", 1, &KittiServer::cloudCB, this);
}

void KittiServer::advertise()
{
    pubMap_ = nh_.advertise<sensor_msgs::PointCloud2>(
                "octree_map", 1);
}

void KittiServer::updateMap(const point3d &sensorOrigin)
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

          if (sqrt(pow(it.getX() - sensorOrigin.x(),2) + pow(it.getY() - sensorOrigin.y(),2)) >= MAP_RADIUS)
              continue;
          point3d old_point(it.getX(), it.getY(), it.getZ());

          m_octree->updateNode(old_point, true);
        }
    }
}

void KittiServer::cloudCB(const sensor_msgs::PointCloud2Ptr &points)
{
    /// Step1 Obtain the Pointcloud
    ros::WallTime startTime = ros::WallTime::now();
    PointCloud pc; // input cloud for filtering and ground-detection
    pcl::fromROSMsg(*points, pc);
    ROS_INFO_STREAM("Before filtering " << pc.size());

    /// vox filter
    PointCloudPtr voxIn (new PointCloud);
    *voxIn = pc;
    pcl::VoxelGrid<PointT> vox_filter;
    vox_filter.setInputCloud(voxIn);
    vox_filter.setLeafSize(0.3, 0.3, 0.3);
    vox_filter.filter(pc);
    ROS_INFO_STREAM("After filtering " << pc.size());

    PointCloud cloud;
    /// Filter the neighbor noise
    for (size_t i=0; i < pc.size(); i++){
        PointT point = pc.points[i];
        if (fabs(point.x) < 2 && fabs(point.y) < 2)
            continue;
        cloud.push_back(point);
    }

    /// Step2 Extract the transformation from tf
    tf::StampedTransform sensorToWorldTf;
    try {
      m_tfListener.lookupTransform(m_worldFrameId, points->header.frame_id, points->header.stamp, sensorToWorldTf);
    } catch(tf::TransformException& ex){
      return;
    }
    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    /// Step3 If the pose has gone far enough, reset the map
    static Eigen::Matrix4f previousLong = Eigen::MatrixXf::Zero(4, 4);
    double distance = sqrt(pow(sensorToWorld(0,3) - previousLong(0,3),2) +
                           pow(sensorToWorld(1,3) - previousLong(1,3),2));
    if (distance > 4) {
        point3d sensor_org (sensorToWorld(0,3), sensorToWorld(1,3), sensorToWorld(2,3));
        updateMap(sensor_org);
        previousLong = sensorToWorld;
    }

    pcl::transformPointCloud(cloud, cloud, sensorToWorld);

    /// Step6 Insert the Time Labeled LaserScan
    insertPC(sensorToWorld, cloud);
    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_INFO("Pointcloud insertion in MapServer done (%zu pts, %f sec)", cloud.size(), total_elapsed);
    publishAll(points->header.stamp);
}

void KittiServer::insertPC(const Eigen::Matrix4f &trans, const PCLPointCloud &pc)
{
    ROS_INFO("detected points and begin to insert");

    //! Step1 Extract out each occupied cells
    KeySet free_cells, occupied_cells;
    point3d pOri(trans(0,3), trans(1, 3), trans(2, 3));

    // free on ray, occupied on endpoint:
    for (PCLPointCloud::const_iterator it = pc.begin(); it != pc.end(); ++it){
      point3d point(it->x, it->y, it->z);
      // maxrange check
      if ((m_maxRange < 0.0) || ((point - pOri).norm() <= m_maxRange) ) {

        // free cells
        if (m_octree->computeRayKeys(pOri, point, m_keyRay)){
          free_cells.insert(m_keyRay.begin(), m_keyRay.end());
        }
        // occupied endpoint
        OcTreeKey key;
        if (m_octree->coordToKeyChecked(point, key)){
          occupied_cells.insert(key);

          updateMinKey(key, m_updateBBXMin);
          updateMaxKey(key, m_updateBBXMax);
        }
      } else {// ray longer than maxrange:;
        point3d new_end = pOri + (point - pOri).normalized() * m_maxRange;
        if (m_octree->computeRayKeys(pOri, new_end, m_keyRay)){
          free_cells.insert(m_keyRay.begin(), m_keyRay.end());

          octomap::OcTreeKey endKey;
          if (m_octree->coordToKeyChecked(new_end, endKey)){
            free_cells.insert(endKey);
            updateMinKey(endKey, m_updateBBXMin);
            updateMaxKey(endKey, m_updateBBXMax);
          } else{
            ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
          }
        }
      }
    }

    // mark free cells only if not seen occupied in this cloud
    for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
      if (occupied_cells.find(*it) == occupied_cells.end()){
            m_octree->updateNode(*it, false);
      }
    }

    // now mark all occupied cells:
    for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
        point3d point = m_octree->keyToCoord(*it);
        if ((point - pOri).norm() > MAP_RADIUS) continue;
        m_octree->updateNode(*it, true);
    }

}
