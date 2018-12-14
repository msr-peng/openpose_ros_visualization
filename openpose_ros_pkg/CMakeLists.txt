cmake_minimum_required(VERSION 2.8.7)
project(openpose_ros_pkg)

add_compile_options(-std=c++11)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

find_package(GFlags)
find_package(Glog)
find_package(OpenCV 3.3.1 EXACT REQUIRED)
find_package(OpenPose REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	std_msgs
	sensor_msgs
	roscpp
	cv_bridge
	openpose_ros_msgs
	roslib
	image_transport
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
  CATKIN_DEPENDS std_msgs 
  roscpp
  cv_bridge
  openpose_ros_msgs  	
  roslib
  image_transport			 
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OPENPOSE_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(/root/src/caffe/build/install/lib)

add_executable(openpose_ros_node_3d src/openpose_ros_node_3d.cpp)
target_link_libraries(openpose_ros_node_3d
${catkin_LIBRARIES}

${OpenPose_LIBS} 
													${GFLAGS_LIBRARY}
													${OpenCV_LIBS} 
													${GLOG_LIBRARY})
						
add_dependencies(openpose_ros_node_3d ${catkin_EXPORTED_TARGETS})
