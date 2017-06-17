#pragma once
#include "map_server/DynamicServer.h"
#include "common.h"

//#define DEBUG

using namespace map_server;

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
          if (sqrt(pow(it.getX() - sensorOrigin.x(),2) + pow(it.getY() - sensorOrigin.y(),2)) >= map_scale_)
              continue;
          point3d old_point(it.getX(), it.getY(), it.getZ());
          m_octree->updateNode(old_point, true);
        }
    }
}

void DynamicServer::loamCallback(const sensor_msgs::PointCloud2& loam)
{
    /// Step1 Obtain the Pointcloud
    ros::WallTime startTime = ros::WallTime::now();
    PointCloudPtr pc (new PointCloud); // input cloud for filtering and ground-detection
    pcl::fromROSMsg(loam, *pc);

    /// Step2 Cut the Pointcloud
    ROS_INFO_STREAM("old pc data "<< pc->size());
    PointCloudPtr npc (new PointCloud);
    PointCloudCut(pc, *npc);
    ROS_INFO_STREAM("new pc data "<< npc->size());


    /// Step3 vox filter
    PointCloudPtr voxIn (new PointCloud);
    *voxIn = *npc;
    pcl::VoxelGrid<PointT> vox_filter;
    vox_filter.setInputCloud(voxIn);
    vox_filter.setLeafSize(m_res+0.1, m_res+0.1, m_res+0.1);
    vox_filter.filter(*pc);
    ROS_INFO_STREAM("voxel filtered points "<< pc->size());

    /// Step4 split into ground and nonground
    PointCloud pc_ground, pc_nonground;
    groundFilter(pc, pc_ground, pc_nonground);
#if DEBUG
    ROS_INFO_STREAM("pc "<< pc->size() << " ground "
                    << pc_ground.size() << " nonground "
                    << pc_nonground.size());
#endif
    /// Step5 Extract the transformation from tf
    double yaw = 0;
    Eigen::Matrix4f curLoc;
    lookTF(loam, curLoc, yaw);
    pcl::transformPointCloud(*pc, *pc, curLoc);
    pcl::transformPointCloud(pc_ground, pc_ground, curLoc);
    pcl::transformPointCloud(pc_nonground, pc_nonground, curLoc);

    /// Step6 If the pose has gone far enough, reset the map
    static Eigen::Matrix4f preLong = Eigen::MatrixXf::Zero(4, 4);
    double distance = sqrt(pow(curLoc(0,3) - preLong(0,3),2) +
                           pow(curLoc(1,3) - preLong(1,3),2));
    if (distance > 4) {
        point3d sensor_org (curLoc(0,3), curLoc(1,3), curLoc(2,3));
        updateMap(sensor_org);
        preLong = curLoc;
    }

    /// Step-1 Update local octree map
    insertPC(curLoc, *pc, m_octree);
    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_INFO("MapServer done (%zu pts, %f sec)", pc->size(), total_elapsed);
    publishCloud(loam.header.stamp, curLoc, yaw);
    return;
}


void DynamicServer::PointCloudCut(const PointCloudPtr& points,
                                  PointCloud& outputs)
{
    outputs.clear();

    for (PCLPointCloud::const_iterator it = points->begin(); it != points->end(); ++it){

        double distance = sqrt(pow(it->x, 2) + pow(it->y, 2));
        if (distance > (map_scale_+10) ||
                distance < 5 || fabs(it->z > 8))
            continue;

        PointT pc_point(it->x, it->y, it->z);
        outputs.push_back(pc_point);
    }
}

void DynamicServer::insertPC(const Eigen::Matrix4f &trans, const PointCloud &pc, OcTreeT *tree)
{
    //! Step1 Extract out each occupied cells
    KeySet free_cells, occupied_cells;
    point3d pOri(trans(0,3), trans(1, 3), trans(2, 3));

    // free on ray, occupied on endpoint:
    for (PCLPointCloud::const_iterator it = pc.begin(); it != pc.end(); ++it){
      point3d point(it->x, it->y, it->z);
      // maxrange check
      if ((m_maxRange < 0.0) || ((point - pOri).norm() <= m_maxRange) ) {

        // free cells
        if (tree->computeRayKeys(pOri, point, m_keyRay)){
          free_cells.insert(m_keyRay.begin(), m_keyRay.end());
        }
        // occupied endpoint
        OcTreeKey key;
        if (tree->coordToKeyChecked(point, key)){
          occupied_cells.insert(key);

          updateMinKey(key, m_updateBBXMin);
          updateMaxKey(key, m_updateBBXMax);
        }
      } else {// ray longer than maxrange:;
        point3d new_end = pOri + (point - pOri).normalized() * m_maxRange;
        if (tree->computeRayKeys(pOri, new_end, m_keyRay)){
          free_cells.insert(m_keyRay.begin(), m_keyRay.end());

          octomap::OcTreeKey endKey;
          if (tree->coordToKeyChecked(new_end, endKey)){
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
            tree->updateNode(*it, false);
      }
    }

    // now mark all occupied cells:
    for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
        point3d point = tree->keyToCoord(*it);
        if ((point - pOri).norm() > map_scale_) continue;
        tree->updateNode(*it, true);
    }

}
