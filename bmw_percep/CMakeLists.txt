cmake_minimum_required(VERSION 2.8.3)
project(bmw_percep)

#non catkin packages
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_include_dirs})

#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_include_dirs})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
INCLUDE_DIRS include
#  LIBRARIES bmw_percep
#  CATKIN_DEPENDS opencv pcl roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(bmw_percep
#   src/${PROJECT_NAME}/bmw_percep.cpp
# )

## Declare a cpp executable
add_executable(publish_kinect 
    src/pcl_cv_utils.cpp
    src/pub_kinect_msgs.cpp)

add_executable(sample_track
  src/particleFilter.cpp
  src/groundPlane.cpp
  src/pcl_cv_utils.cpp
  src/sample_people_track.cpp)

add_executable(sample_track_depth
  src/particleFilter.cpp
  src/groundPlane.cpp
  src/pcl_cv_utils.cpp
  src/sample_people_track_depth.cpp)

add_executable(xyz_based_tracking
  src/particleFilter.cpp
  src/groundPlane.cpp
  src/pcl_cv_utils.cpp
  src/xyz_based_tracking.cpp)

add_executable(cola_demo_tracking
  src/particleFilter.cpp
  src/groundPlane.cpp
  src/pcl_cv_utils.cpp
  src/cola_demo_tracking.cpp)


add_executable(sample_track_gmg
  src/pcl_cv_utils.cpp
  src/sample_people_track_gmg.cpp)

add_executable(sample_ground
  src/groundPlane.cpp
  src/sample_groundPlane.cpp)

add_executable(sample_groundReader
  src/groundPlane.cpp
  src/sample_groundPlaneReader.cpp)

#add_executable(people_pcl src/people_pcl.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(bmw_percep_node bmw_percep_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(bmw_percep_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(publish_kinect ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(sample_track ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(sample_track_depth ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(xyz_based_tracking ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(cola_demo_tracking ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(sample_track_gmg ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(sample_ground ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(sample_groundReader ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS bmw_percep bmw_percep_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bmw_percep.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
