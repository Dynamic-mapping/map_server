cmake_minimum_required(VERSION 2.8)
project(map_server)

add_compile_options(-std=c++11)

set(PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  std_msgs
  std_srvs
  octomap_ros
  octomap_msgs
  dynamic_reconfigure
  nodelet  
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(PCL 1.8.0 EXACT REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(cfg/map_server.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS octomap PCL
)

set(LINK_LIBS
  ${OpenCV_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

FILE(GLOB_RECURSE SRC_LIST    "src/*.cpp")
FILE(GLOB_RECURSE HEADER_LIST "include/map_server/*.h" "include/map_server/impl/*.hpp" "include/common.h")

link_directories(${PCL_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_executable(dynamic_node src/dynamic_node.cpp ${HEADER_LIST})
target_link_libraries(dynamic_node ${LINK_LIBS})
