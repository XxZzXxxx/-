cmake_minimum_required(VERSION 3.0.2)
project(shared_msgs)

find_package(catkin REQUIRED COMPONENTS
message_generation
std_msgs
geometry_msgs
)

add_message_files(
  FILES
  TaskResult.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime 
  std_msgs 
  geometry_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


