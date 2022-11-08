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


pkg_check_modules(LIBNOTIFY QUIET libnotify)

find_path(LIBNOTIFY_INCLUDE_DIRS
    NAMES notify.h
    HINTS ${LIBNOTIFY_INCLUDEDIR}
          ${LIBNOTIFY_INCLUDE_DIRS}
    PATH_SUFFIXES libnotify
)

find_library(LIBNOTIFY_LIBRARIES
    NAMES notify
    HINTS ${LIBNOTIFY_LIBDIR}
          ${LIBNOTIFY_LIBRARY_DIRS}
)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LibNotify REQUIRED_VARS LIBNOTIFY_INCLUDE_DIRS LIBNOTIFY_LIBRARIES
                                            VERSION_VAR   LIBNOTIFY_VERSION)

mark_as_advanced(
    LIBNOTIFY_INCLUDE_DIRS
    LIBNOTIFY_LIBRARIES
)

message(${LIBNOTIFY_INCLUDE_DIRS})


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
  ${LIBNOTIFY_INCLUDE_DIRS}
)


### localizer_node的是需要從topic接收map的
### icp_localization是直接讀取map.pcd的

add_executable(base_localizer src/localizer_node.cpp)
target_link_libraries(base_localizer ${catkin_LIBRARIES} )


add_executable(localizer_no_pcd src/localizer_node2.cpp)
target_link_libraries(localizer_no_pcd ${catkin_LIBRARIES} )

add_executable(icp1 src/icp_locolization1.cpp)
target_link_libraries(icp1 ${catkin_LIBRARIES})

add_executable(icp2 src/icp_locolization2.cpp)
target_link_libraries(icp2 ${catkin_LIBRARIES})


add_executable(icp3 src/icp_locolization3.cpp)
target_link_libraries(icp3 ${catkin_LIBRARIES})


add_executable(pub_map src/pub_map_node.cpp)
target_link_libraries(pub_map ${catkin_LIBRARIES})

