cmake_minimum_required(VERSION 2.8.3)
project(decision_tree)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
  traj_utils
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES decision_tree 
 CATKIN_DEPENDS plan_env traj_utils
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_executable( decision_tree 
    src/decistion.cpp
    )
target_link_libraries( decision_tree
    ${catkin_LIBRARIES} 
    )  
