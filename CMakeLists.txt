cmake_minimum_required(VERSION 2.8.3)
project(agile_grasp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cmake_modules cv_bridge eigen_conversions geometry_msgs 
message_generation pcl_conversions roscpp sensor_msgs std_msgs visualization_msgs)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)
find_package(LAPACK REQUIRED)
find_package(OpenCV)
find_package(PCL REQUIRED)
#~ find_package(VTK 6.0 REQUIRED NO_MODULE) # required in Fedora Core

## Set compiler optimization flags
set(CMAKE_CXX_FLAGS "-DNDEBUG -O3 -fopenmp -Wno-deprecated -Wenum-compare")

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
add_message_files(FILES CloudSized.msg Grasp.msg Grasps.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

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
#  INCLUDE_DIRS include
#  LIBRARIES grasp_affordances
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
 INCLUDE_DIRS 
  include
 LIBRARIES
  quadric
  finger_hand
  rotating_hand
  antipodal
  localization
  learning
  handle
  grasp_localizer
  handle_search
  grasp_hypothesis
  plot
  hand_search
 CATKIN_DEPENDS
	cmake_modules
	cv_bridge
  eigen_conversions 
  geometry_msgs 
  message_runtime 
  pcl_conversions
  roscpp 
  sensor_msgs
  std_msgs
  visualization_msgs
 DEPENDS 
  Eigen
  LAPACK
  OpenCV
  PCL  
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#~ include(${VTK_USE_FILE}) # required in Fedora Core

## Declare a cpp library
add_library(quadric src/${PROJECT_NAME}/quadric.cpp)
add_library(finger_hand src/${PROJECT_NAME}/finger_hand.cpp)
add_library(rotating_hand src/${PROJECT_NAME}/rotating_hand.cpp)
add_library(antipodal src/${PROJECT_NAME}/antipodal.cpp)
add_library(localization src/${PROJECT_NAME}/localization.cpp)
add_library(learning src/${PROJECT_NAME}/learning.cpp)
add_library(handle src/${PROJECT_NAME}/handle.cpp)
add_library(grasp_localizer src/${PROJECT_NAME}/grasp_localizer.cpp)
add_library(handle_search src/${PROJECT_NAME}/handle_search.cpp)
add_library(grasp_hypothesis src/${PROJECT_NAME}/grasp_hypothesis.cpp)
add_library(plot src/${PROJECT_NAME}/plot.cpp)
add_library(hand_search src/${PROJECT_NAME}/hand_search.cpp)

## Declare a cpp executable
add_executable(test_taubin_fitting src/tests/test_taubin.cpp)
add_executable(hands_test src/tests/hands_test.cpp)
add_executable(antipodal_test src/tests/antipodal_test.cpp)
add_executable(learning_test src/tests/learning_test.cpp)
add_executable(test_local_axes src/tests/test_local_axes.cpp)
add_executable(opencv_test src/tests/opencv_test.cpp)
add_executable(train_svm src/nodes/train.cpp)
add_executable(test_svm src/nodes/test.cpp)
add_executable(find_grasps src/nodes/find_grasps.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(grasp_localizer_node grasp_localizer_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(quadric ${PCL_LIBRARIES} lapack)
target_link_libraries(rotating_hand antipodal finger_hand grasp_hypothesis ${PCL_LIBRARIES})
target_link_libraries(localization grasp_hypothesis hand_search handle handle_search learning plot ${PCL_LIBRARIES})
target_link_libraries(test_taubin_fitting quadric ${PCL_LIBRARIES})
target_link_libraries(hands_test quadric rotating_hand finger_hand ${PCL_LIBRARIES})
target_link_libraries(antipodal_test localization quadric rotating_hand finger_hand ${PCL_LIBRARIES})
target_link_libraries(test_local_axes localization)
target_link_libraries(learning rotating_hand ${OpenCV_LIBRARIES})
target_link_libraries(learning_test learning localization quadric rotating_hand finger_hand ${PCL_LIBRARIES})
target_link_libraries(train_svm learning localization ${PCL_LIBRARIES})
target_link_libraries(test_svm learning localization ${PCL_LIBRARIES})
target_link_libraries(opencv_test ${OpenCV_LIBRARIES})
target_link_libraries(handle rotating_hand)
target_link_libraries(find_grasps grasp_localizer ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(grasp_localizer handle localization rotating_hand ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(handle_search handle rotating_hand ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(plot grasp_hypothesis handle quadric ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(hand_search finger_hand grasp_hypothesis plot quadric rotating_hand ${PCL_LIBRARIES})

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE)

## install launch files
install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

