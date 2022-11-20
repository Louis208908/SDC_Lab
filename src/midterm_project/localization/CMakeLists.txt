cmake_minimum_required(VERSION 3.0.2)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf2
  tf2_msgs
  tf_conversions
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localization 
#  CATKIN_DEPENDS geometry_msgs pcl_ros roscpp rospy sensor_msgs tf2 tf2_msgs tf_conversions
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


set(lib_DIR /usr/lib/x84_64-linux-gnu)
link_directories(${lib_DIR})

### localizer_node的是需要從topic接收map的
### icp_localization是直接讀取map.pcd的

add_executable(base_localizer src/localizer_node.cpp)
target_link_libraries(base_localizer ${catkin_LIBRARIES} )


add_executable(localizer_no_pcd src/localizer_node2.cpp)
target_link_libraries(localizer_no_pcd ${catkin_LIBRARIES} libjsoncpp.a)

add_executable(icp1 src/icp_locolization1.cpp)
target_link_libraries(icp1 ${catkin_LIBRARIES})

add_executable(icp2 src/icp_locolization2.cpp)
target_link_libraries(icp2 ${catkin_LIBRARIES})


add_executable(icp3 src/icp_locolization3.cpp)
target_link_libraries(icp3 ${catkin_LIBRARIES})


add_executable(icp_ekf src/icp_locolization_ekf.cpp)
target_link_libraries(icp_ekf ${catkin_LIBRARIES})

add_executable(merge_pcd src/mergePCD.cpp)
target_link_libraries(merge_pcd ${catkin_LIBRARIES})


add_executable(pub_map src/pub_map_node.cpp)
target_link_libraries(pub_map ${catkin_LIBRARIES})

