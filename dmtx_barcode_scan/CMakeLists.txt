cmake_minimum_required(VERSION 2.8.3)
project(dmtx_barcode_scan)

SET (PROJECT_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake)
SET (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${PROJECT_CMAKE_DIR}")
#SET (DMTX_DIR ${PROJECT_SOURCE_DIR}/cmake)

#find_package (DMTX)

#find_library(DMTX_LIBRARY dmtx PATH /usr/lib/)

#if( DMTX_FOUND )
find_package (DMTX)
  ## Find catkin macros and libraries
  ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  ## is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    message_generation
  )
  
  find_package(OpenCV REQUIRED)

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Scan.msg
#   Message2.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)
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
  #  LIBRARIES dmtx_barcode_scan
  CATKIN_DEPENDS cv_bridge image_transport opencv2 message_runtime
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
   ${DMTX_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  
  add_executable(barcode_scan_node src/barcode_scan_node.cpp)

  add_executable(barcodeScan src/barcodeScan.cpp)
  
  target_link_libraries(barcode_scan_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${DMTX_LIBRARIES}
  )

  target_link_libraries(barcodeScan
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${DMTX_LIBRARIES}
  )

#endif()

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
# install(TARGETS dmtx_barcode_scan dmtx_barcode_scan_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dmtx_barcode_scan.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
