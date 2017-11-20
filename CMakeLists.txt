cmake_minimum_required(VERSION 3.1)
project(pcload)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
        rosbag
        sensor_msgs
        geometry_msgs
        #eigen_conversions
        #pcl-all-dev
        #mavros_msgs
	#cmake_modules
)

find_package(PCL 1.7 REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES mavros
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs rosbag #eigen_conversions
  DEPENDS PCL Eigen3
)

include_directories(
	${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)

add_executable(pcload main.cpp)
#add_dependencies(gcs_agent mavros_msgs)
#add_dependencies(gcs_agent mavros_msgs_generate_messages_cpp)
#add_dependencies(pcload geometry_msgs_generate_messages_cpp)

target_link_libraries(pcload
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
