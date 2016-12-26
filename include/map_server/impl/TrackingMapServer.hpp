#pragma once
#include "map_server/TrackingMapServer.h"
#include <string>

using namespace octomap;

namespace map_server {

TrackingMapServer::TrackingMapServer(const std::string& filename) :
        MapServer()
{
  //read tree if necessary
  if (filename != "") {
    if (m_octree->readBinary(filename)) {
      ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(), m_octree->size());
      m_treeDepth = m_octree->getTreeDepth();
      m_res = m_octree->getResolution();
      m_gridmap.info.resolution = m_res;

      publishAll();
    } else {
      ROS_ERROR("Could not open requested file %s, exiting.", filename.c_str());
      exit(-1);
    }
  }

  ros::NodeHandle private_nh("~");

  std::string changeSetTopic = "changes";
  std::string changeIdFrame = "/talker/changes";

  private_nh.param("topic_changes", changeSetTopic, changeSetTopic);
  private_nh.param("change_id_frame", change_id_frame, changeIdFrame);
  private_nh.param("track_changes", track_changes, false);
  private_nh.param("listen_changes", listen_changes, false);
  private_nh.param("min_change_pub", min_change_pub, 0);

  if (track_changes && listen_changes) {
    ROS_WARN("MapServer: It might not be useful to publish changes and at the same time listen to them."
        "Setting 'track_changes' to false!");
    track_changes = false;
  }

  if (track_changes) {
    ROS_INFO("starting server");
    pubChangeSet = private_nh.advertise<sensor_msgs::PointCloud2>(
        changeSetTopic, 1);
    m_octree->enableChangeDetection(true);
  }

  if (listen_changes) {
    ROS_INFO("starting client");
    subChangeSet = private_nh.subscribe(changeSetTopic, 1,
                                        &TrackingMapServer::trackCallback, this);
  }
}

TrackingMapServer::~TrackingMapServer() {
}

void TrackingMapServer::insertScan(const tf::Point & sensorOrigin, const PCLPointCloud & ground, const PCLPointCloud & nonground) {
  MapServer::insertScan(sensorOrigin, ground, nonground);

  if (track_changes) {
    trackChanges();
  }
}

void TrackingMapServer::trackChanges() {
  KeyBoolMap::const_iterator startPnt = m_octree->changedKeysBegin();
  KeyBoolMap::const_iterator endPnt = m_octree->changedKeysEnd();

  pcl::PointCloud<pcl::PointXYZI> changedCells = pcl::PointCloud<pcl::PointXYZI>();

  int c = 0;
  for (KeyBoolMap::const_iterator iter = startPnt; iter != endPnt; ++iter) {
    ++c;
    OcTreeNode* node = m_octree->search(iter->first);

    bool occupied = m_octree->isNodeOccupied(node);

    point3d center = m_octree->keyToCoord(iter->first);

    pcl::PointXYZI pnt;
    pnt.x = center(0);
    pnt.y = center(1);
    pnt.z = center(2);

    if (occupied) {
      pnt.intensity = 1000;
    }
    else {
      pnt.intensity = -1000;
    }

    changedCells.push_back(pnt);
  }

  if (c > min_change_pub)
  {
    sensor_msgs::PointCloud2 changed;
    pcl::toROSMsg(changedCells, changed);
    changed.header.frame_id = change_id_frame;
    changed.header.stamp = ros::Time().now();
    pubChangeSet.publish(changed);
    ROS_DEBUG("[server] sending %d changed entries", (int)changedCells.size());

    m_octree->resetChangeDetection();
    ROS_DEBUG("[server] octomap size after updating: %d", (int)m_octree->calcNumNodes());
  }
}

void TrackingMapServer::trackCallback(sensor_msgs::PointCloud2Ptr cloud) {
  pcl::PointCloud<pcl::PointXYZI> cells;
  pcl::fromROSMsg(*cloud, cells);
  ROS_DEBUG("[client] size of newly occupied cloud: %i", (int)cells.points.size());

  for (size_t i = 0; i < cells.points.size(); i++) {
    pcl::PointXYZI& pnt = cells.points[i];
    m_octree->updateNode(m_octree->coordToKey(pnt.x, pnt.y, pnt.z), pnt.intensity, false);
  }

  m_octree->updateInnerOccupancy();
  ROS_DEBUG("[client] octomap size after updating: %d", (int)m_octree->calcNumNodes());
}

} /* namespace octomap */
