#pragma once
#include "map_server/DynamicServer.h"
#include "../../common.h"

#define DEBUG

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

    /// Check the Old Octree
    OcTreeT oldObjTree = *obj_scan;
    obj_scan->clear();
    for (OcTree::leaf_iterator it = oldObjTree.begin(); it != oldObjTree.end(); it++) {
        if(oldObjTree.isNodeOccupied(*it)){
          if (sqrt(pow(it.getX() - sensorOrigin.x(),2) + pow(it.getY() - sensorOrigin.y(),2)) >= map_scale_)
              continue;
          point3d old_point(it.getX(), it.getY(), it.getZ());
          obj_scan->updateNode(old_point, true);
        }
    }
}

void DynamicServer::loamCallback(const doom::LoamScanPtr& loam)
{
    /// Step1 Obtain the Pointcloud reduce to circle area
    ros::WallTime startTime = ros::WallTime::now();
    PointCloudPtr pc (new PointCloud); // input cloud for filtering and ground-detection
    pcl::fromROSMsg(loam->cloud, *pc);

    ROS_INFO_STREAM("old pc data "<< pc->size());
    PointCloudPtr npc (new PointCloud);
    PointCloudCut(pc, *npc);
    ROS_INFO_STREAM("new pc data "<< npc->size());

    /// Step2 vox filter
    PointCloudPtr vox (new PointCloud);
    *vox = *npc;
    voxFilter(vox, m_res+0.1);
    *npc = *vox;

    if (npc->size() <= 100)
    {
        ROS_ERROR_STREAM("PC points too small " << npc->size());
        return;
    }
    /// Step3 split into ground and nonground
    PointCloud pc_ground, pc_nonground;
    groundFilter(npc, pc_ground, pc_nonground);

    /// Step4 Transform the pointcloud into the world frame
    double yaw = 0;
    Eigen::Matrix4f curLoc;
    lookTF(loam, curLoc, yaw);
    pcl::transformPointCloud(*npc, *npc, curLoc);
    pcl::transformPointCloud(pc_ground, pc_ground, curLoc);
    pcl::transformPointCloud(pc_nonground, pc_nonground, curLoc);
    /// Step5 Get approximate nearest neighbors from the octree map
    PointCloudPtr pc_neighbors (new PointCloud);
    ApproxNearestNeighbors(npc, *pc_neighbors);

    if (pc_neighbors->size() >= 100)
    {
#ifdef DEBUG
        ROS_INFO_STREAM("all "<< pc->size() << ", neighbors "
                    << pc_neighbors->size());
#endif

        /// Step6 update the input pointclouds with the neighbors
//        MeasurementUpdate(pc, pc_neighbors, *pc);
#ifdef DEBUG
        ROS_INFO_STREAM("pc "<< pc->size() << ", ground "
                    << pc_ground.size() << ", nonground "
                    << pc_nonground.size());
#endif
    }

    // publish Color Points
    sensor_msgs::PointCloud2 msg_ground, msg_nonground;
    pcl::toROSMsg (pc_ground, msg_ground);
    pcl::toROSMsg (pc_nonground, msg_nonground);
    msg_ground.header.frame_id = "world";
    msg_nonground.header.frame_id = "world";
    msg_ground.header.stamp = loam->header.stamp;
    msg_nonground.header.stamp = loam->header.stamp;
    pub_ground.publish(msg_ground);
    pub_nonground.publish(msg_nonground);


    /// Step5 If the pose has gone far enough, reset the map
    static Eigen::Matrix4f preLong = Eigen::MatrixXf::Zero(4, 4);
    double distance = sqrt(pow(curLoc(0,3) - preLong(0,3),2) +
                           pow(curLoc(1,3) - preLong(1,3),2));
    if (distance > 4) {
        point3d sensor_org (curLoc(0,3), curLoc(1,3), curLoc(2,3));
        updateMap(sensor_org);
        preLong = curLoc;
    }

    /// Step6 Check the difference between new single-bin scan and octree map
//    cur_scan->clear();
//    dy_pc.clear();
//    insertPC(curLoc, pc_nonground, cur_scan);
//    checkDiff();

    /// Step7 dynamic points cluster


    /// Step8 series points estimation


    /// Step9 dynamic object estimation

    /// Step 10 Update local octree map
    plane_octree->clear();
    nonplane_octree->clear();
    insertNonGround(curLoc, pc_nonground);
    insertGround(curLoc, pc_ground);
//    insertPC(curLoc, *npc, m_octree);
//    insertTimeScan(curLoc, loam);
    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_INFO("MapServer done (%zu pts, %f sec)", pc->size(), total_elapsed);
    publishCloud(loam->header.stamp, curLoc, yaw);
    return;
}

void DynamicServer::insertNonGround(const Eigen::Matrix4f &trans, const PointCloud &pc)
{
    //! Step1 Extract out each occupied cells
    KeySet occupied_cells;
    point3d pOri(trans(0,3), trans(1, 3), trans(2, 3));

    // free on ray, occupied on endpoint:
    for (PCLPointCloud::const_iterator it = pc.begin(); it != pc.end(); ++it){
      point3d point(it->x, it->y, it->z);

      if (((point - pOri).norm() <= 4) || ((point - pOri).norm() >= 100))
          continue;

      OcTreeKey key;
      if (obj_scan->coordToKeyChecked(point, key))
        occupied_cells.insert(key);
    }

    // now mark all occupied cells:
    for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
        point3d point = obj_scan->keyToCoord(*it);
        if ((point - pOri).norm() > map_scale_) continue;
        obj_scan->updateNode(*it, true);
    }

    // Extract the Lowerest point in object scan octree
    float sum_z = 0.0;
    int num_z = 1.0;
    for(OcTree::iterator it = obj_scan->begin(); it != obj_scan->end(); ++it){
        if (obj_scan->isNodeOccupied(*it)){
            point3d pt = it.getCoordinate();
            sum_z += pt.z();
            num_z += 1;
        }
    }
    float sub_z = sum_z/num_z;
    ROS_INFO_STREAM ("min z is " << sub_z);

    // Lower all the nonground
    for(OcTree::iterator it = obj_scan->begin(); it != obj_scan->end(); ++it){
        point3d pt = it.getCoordinate();
        pt.z() -= sub_z-1.5;
        if (pt.z() > 0) nonplane_octree->updateNode(pt, true);
    }
}

void DynamicServer::insertGround(const Eigen::Matrix4f &trans, const PointCloud &pc)
{
    //! Step1 Extract out each occupied cells
    KeySet occupied_cells;
    point3d pOri(trans(0,3), trans(1, 3), trans(2, 3));

    // free on ray, occupied on endpoint:
    for (PCLPointCloud::const_iterator it = pc.begin(); it != pc.end(); ++it){
      point3d point(it->x, it->y, it->z);

      if (((point - pOri).norm() <= 4) || ((point - pOri).norm() >= 100))
          continue;

      OcTreeKey key;
      if (m_octree->coordToKeyChecked(point, key))
        occupied_cells.insert(key);
    }

    // now mark all occupied cells:
    for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
        point3d point = m_octree->keyToCoord(*it);
        if ((point - pOri).norm() > map_scale_) continue;
        m_octree->updateNode(*it, true);
    }

    // Extract plane octree map
    for(OcTree::iterator it = m_octree->begin(); it != m_octree->end(); ++it){
        if (plane_octree->isNodeOccupied(*it)){
            point3d pt = it.getCoordinate();
            pt.z() = 0;
            plane_octree->updateNode(pt, true);
        }
    }
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

void DynamicServer::MeasurementUpdate(const PointCloudPtr& points,
                                      const PointCloudPtr& neighbors,
                                      PointCloud& aligned)
{
    // G-ICP based alignment.
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setRANSACIterations(0);
    icp.setMaximumIterations(10);

    icp.setInputSource(points);
    icp.setInputTarget(neighbors);

    icp.align(aligned);
}


void DynamicServer::ApproxNearestNeighbors(const PointCloudPtr & points, PointCloud& neighbors){

    neighbors.points.clear();
    // Iterate over points in the input point cloud, finding the nearest neighbor
    // for every point and storing it in the output array.
    for (PCLPointCloud::const_iterator it = points->begin(); it != points->end(); ++it){

        point3d pt(it->x, it->y, it->z);

        // Search the point in the octree
        OcTreeNode* node = m_octree->search(pt);
        if (node){
            PointT pc_point(pt.x(), pt.y(), pt.z());
            neighbors.push_back(pc_point);
        }
    }
}

void DynamicServer::insertTimeScan(const Eigen::Matrix4f &trans, const doom::LoamScanPtr& loam)
{
    //! Step1 Extract out each occupied cells
    KeySet occupied_cells;
    point3d pOri(trans(0,3), trans(1, 3), trans(2, 3)-CAR_HEIGHT);

    for(size_t i = 0 ; i < loam->Scans.size(); i++) {

        doom::LaserScan scan = loam->Scans[i];

//#ifdef DEBUG
//        ROS_INFO_STREAM("" << i << " " << scan.Points.size());
//        for (size_t j = 0; j < scan.Points.size(); j++) {
//            std::cout << j << " " << scan.Points[j].x << " " << scan.Points[j].y << " " << scan.Points[j].z << std::endl;
//        }
//        ROS_INFO_STREAM("========================================");
//        continue;
//#endif

        /// Note for the velodyne64 laser, the in coming data is 1->3, -24->0 in degree
        /// The closest point is at point[3], here we only use the points on the plane
        /// Angle from -24 degree to -10 degree

        //! 1.1 Find the first hit points
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
        for (size_t j = start_id; j < 27; j++) {

            //! 1.2.1 not valid for point j
            if (fabs(scan.Points[j].x) <= 0.001) {
                continue;
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
                if(pNorm.norm() > (map_scale_ - (pStart - pOri).norm())) {

                    if (j==26)
                        pEnd = pStart + pNorm.normalize() * (map_scale_ - (pStart - pOri).norm());
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
    //! Step2 Upadte Occupied Cells
    for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {

        point3d point = m_octree->keyToCoord(*it);
        if ((point - pOri).norm() > map_scale_) continue;

        m_octree->updateNode(*it, true);

        // Update upper and lower
        for (size_t h = -3; h < 3; h++) {
            point = m_octree->keyToCoord(*it);
            point.z() += h * 0.8;
            m_octree->updateNode(point, false);
        }
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

//    // mark free cells with the pre_octree
//    for(OcTree::iterator it = cur_scan->begin(); it != cur_scan->end(); ++it){
//        if (!cur_scan->isNodeOccupied(*it)){
//            point3d pt = it.getCoordinate();
//            OcTreeNode* node = tree->search(pt);
//            if (node && tree->isNodeOccupied(node)){
//                OcTreeKey key;
//                if (tree->coordToKeyChecked(pt, key)){
//                  free_cells.insert(key);
//                }
//            }
//        }
//    }

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

void DynamicServer::checkDiff(void)
{

    /// Potential status of pre and current occupancy state
    /// St-1   St   State
    /// 0      0    Nothing
    /// 0      1    Unknown
    /// 1      0    Potential dynamics
    /// 1      1    Potential Statics
    // mark free cells with the pre_octree
    PointCloudPtr dy_points (new PointCloud);
//    for(OcTree::iterator it = m_octree->begin(); it != m_octree->end(); ++it){

//        if (!m_octree->isNodeOccupied(*it)){

//            point3d pt = it.getCoordinate();
//            OcTreeNode* node = cur_scan->search(pt);
//            if (node && cur_scan->isNodeOccupied(node)){
//                PointT pc_point(pt.x(), pt.y(), pt.z());
//                dy_points->push_back(pc_point);
//            }
//        }
//    }

    ROS_INFO_STREAM("Dynamic points " << dy_points->size());
    if (dy_points->size() < 10)
        return;

    // Create kdtree for object search
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(dy_points);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(2.0); // 2.0m
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(10);
    ec.setSearchMethod(tree);
    ec.setInputCloud(dy_points);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); it++){
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end();++pit)
            dy_pc.push_back(dy_points->points[*pit]);
    }
}
