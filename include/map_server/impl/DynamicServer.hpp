#pragma once
#include "map_server/DynamicServer.h"
#include "common.h"

//#define DEBUG

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
                "/dynamicMapping", 1, &DynamicServer::loamCallback, this);
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

          if (sqrt(pow(it.getX() - sensorOrigin.x(),2) + pow(it.getY() - sensorOrigin.y(),2)) >= MAP_RADIUS)
              continue;
          point3d old_point(it.getX(), it.getY(), it.getZ());

          m_octree->updateNode(old_point, true);
        }
    }
}

void DynamicServer::loamCallback(const doom::LoamScanPtr& loam)
{
    /// Step1 Obtain the Pointcloud
    ros::WallTime startTime = ros::WallTime::now();
    PointCloud pc; // input cloud for filtering and ground-detection
    pcl::fromROSMsg(loam->cloud, pc);
    ROS_INFO_STREAM("Before filtering " << pc.size());

    /// vox filter
    PointCloudPtr voxIn (new PointCloud);
    *voxIn = pc;
    pcl::VoxelGrid<PointT> vox_filter;
    vox_filter.setInputCloud(voxIn);
    vox_filter.setLeafSize(0.3, 0.3, 0.3);
    vox_filter.filter(pc);
    ROS_INFO_STREAM("After filtering " << pc.size());

    /// Step2 Extract the transformation from tf
    tf::StampedTransform sensorToWorldTf;
    try {
      m_tfListener.lookupTransform(m_worldFrameId, loam->header.frame_id, loam->header.stamp, sensorToWorldTf);
    } catch(tf::TransformException& ex){
      return;
    }
    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    ROS_INFO_STREAM("new data with tf");
    /// Step3 If the pose has gone far enough, reset the map
    static Eigen::Matrix4f previousLong = Eigen::MatrixXf::Zero(4, 4);
    double distance = sqrt(pow(sensorToWorld(0,3) - previousLong(0,3),2) +
                           pow(sensorToWorld(1,3) - previousLong(1,3),2));
    if (distance > 4) {
        point3d sensor_org (sensorToWorld(0,3), sensorToWorld(1,3), sensorToWorld(2,3));
        updateMap(sensor_org);
        previousLong = sensorToWorld;
    }
    pcl::transformPointCloud(pc, pc, sensorToWorld);

    /// Step5 Insert Pointcloud
    insertPC(sensorToWorld, pc);

    /// Step6 Insert the Time Labeled LaserScan
    //insertTimeScan(sensorToWorld, loam);
    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_INFO("Pointcloud insertion in MapServer done (%zu pts, %f sec)", pc.size(), total_elapsed);

    publishAll(loam->header.stamp);

//    PointCloud points;
//    for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth), end=m_octree->end(); it != end; ++it) {
//        PointT point;
//        point.x = it.getX();
//        point.y = it.getY();
//        point.z = it.getZ();
//        points.push_back(point);
//    }

//    sensor_msgs::PointCloud2 cloud_msg;
//    pcl::toROSMsg(pc, cloud_msg);
//    cloud_msg.header.frame_id = "world";
//    cloud_msg.header.stamp = loam->header.stamp;
//    pubCenterMap.publish(cloud_msg);
}

void DynamicServer::insertPC(const Eigen::Matrix4f &trans, const PointCloud &pc)
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

void DynamicServer::insertTimeScan(const Eigen::Matrix4f &trans, const doom::LoamScanPtr& loam)
{
    //! Step1 Extract out each occupied cells
    KeySet occupied_cells;
    point3d pOri(trans(0,3), trans(1, 3), trans(2, 3));
    for(size_t i = 0 ; i < loam->Scans.size(); i++) {

        doom::LaserScan scan = loam->Scans[i];

#ifdef DEBUG
        ROS_INFO_STREAM("" << i << " " << scan.Points.size());
        for (size_t j = 0; j < scan.Points.size(); j++) {
            std::cout << j << " " << scan.Points[j].x << " " << scan.Points[j].y << " " << scan.Points[j].z << std::endl;
        }
        ROS_INFO_STREAM("========================================");
        continue;
#endif

        /// Note for the velodyne64 laser, the in coming data is 1->3, -24->0 in degree
        /// The closest point is at point[3], here we ignore the points in degree 1, 2, 3
        //! 1.1 If first point is missing, skip
        Eigen::Vector4f scan_point, result;
        size_t start_id=3;
        for (; start_id<scan.Points.size(); start_id++) {
            if (fabs(scan.Points[start_id].x) <= 0.001) {
                continue;
            } else{
                scan_point = Eigen::Vector4f(scan.Points[start_id].x, scan.Points[start_id].y, scan.Points[start_id].z, 1);
                result = trans * scan_point;
                break;
            }
        }

        //! 1.2 Caculate the Origional Ponint, and first point;
        point3d pStart(result(0), result(1), result(2));
        // If norm_z too bigger
        if ((pStart-pOri).normalize().z() > POINT_NORM)
            continue;

        point3d pEnd;
        for (size_t j = start_id; j < 28; j++) {

            //! 1.2.1 not valid for point j
            if (fabs(scan.Points[j].x) <= 0.001) {
                continue;
                /*
                // If is the end points
                if (j == 6)
                {
                    pEnd = pStart + (pStart - pOri).normalize() * (MAP_RADIUS - (pStart - pOri).norm());
                    if (m_octree->computeRayKeys(pStart, pEnd, m_keyRay)){
                        occupied_cells.insert(m_keyRay.begin(), m_keyRay.end());
                    }

                } else {
                    continue;
                }
                */
            } else
            //! 1.2.1 valid for point j
            {
                scan_point = Eigen::Vector4f(scan.Points[j].x, scan.Points[j].y, scan.Points[j].z, 1);
                result = trans * scan_point;
                pEnd = point3d(result(0), result(1), result(2));

                // Check the vector norm, if the norm_z is better, skip
                point3d pNorm = pEnd-pStart;
                if(pNorm.normalize().z() > POINT_NORM) break;

                // Check whether the pEnd within the map Area.
                if(pNorm.norm() > (MAP_RADIUS - (pStart - pOri).norm())) {

                    if (j==27)
                        pEnd = pStart + pNorm.normalize() * (MAP_RADIUS - (pStart - pOri).norm());
                    else
                        continue;
                }

                // Full the occupancy ceil
                if (m_octree->computeRayKeys(pStart, pEnd, m_keyRay)){
                    occupied_cells.insert(m_keyRay.begin(), m_keyRay.end());
                }

                pStart = pEnd;
            }
        }
    }

    ROS_INFO_STREAM("key size "<< occupied_cells.size());
    //! Step2 Upadte Occupied Cells
    for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {

        point3d point = m_octree->keyToCoord(*it);
//        if ((point - pOri).norm() > MAP_RADIUS) continue;

        m_octree->updateNode(*it, true);

        // Update upper and lower
        for (size_t h = -3; h < 3; h++) {
            point = m_octree->keyToCoord(*it);
            point.z() += h * 0.8;
            m_octree->updateNode(point, false);
        }
    }
    ROS_INFO_STREAM("octree status "<<m_octree->memoryUsageNode() << " "<< m_octree->size());
}
