#pragma once
#include "MapServer.h"

namespace map_server {
class MapServerMultilayer : public MapServer{

public:
  MapServerMultilayer(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
  virtual ~MapServerMultilayer();

protected:
  struct ProjectedMap{
    double minZ;
    double maxZ;
    double z; // for visualization
    std::string name;
    nav_msgs::OccupancyGrid map;
  };
  typedef std::vector<ProjectedMap> MultilevelGrid;

  /// hook that is called after traversing all nodes
  virtual void handlePreNodeTraversal(const ros::Time& rostime);

  /// updates the downprojected 2D map as either occupied or free
  virtual void update2DMap(const OcTreeT::iterator& it, bool occupied);

  /// hook that is called after traversing all nodes
  virtual void handlePostNodeTraversal(const ros::Time& rostime);

  std::vector<ros::Publisher*> m_multiMapPub;
  ros::Subscriber m_attachedObjectsSub;

  std::vector<std::string> m_armLinks;
  std::vector<double> m_armLinkOffsets;

  MultilevelGrid m_multiGridmap;


};
}

#include "impl/MapServerMultilayer.hpp"
