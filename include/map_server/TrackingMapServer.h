#pragma once
#include "MapServer.h"

namespace map_server {

class TrackingMapServer: public MapServer {
public:
  TrackingMapServer(const std::string& filename = "");
  virtual ~TrackingMapServer();

  void trackCallback(sensor_msgs::PointCloud2Ptr cloud);
  void insertScan(const tf::Point& sensorOrigin, const PCLPointCloud& ground, const PCLPointCloud& nonground);

protected:
  void trackChanges();

  bool listen_changes;
  bool track_changes;
  int min_change_pub;
  std::string change_id_frame;
  ros::Publisher pubFreeChangeSet;
  ros::Publisher pubChangeSet;
  ros::Subscriber subChangeSet;
  ros::Subscriber subFreeChanges;
};

} /* namespace octomap */

#include "impl/TrackingMapServer.hpp"
