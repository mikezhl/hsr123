cmake_minimum_required(VERSION 3.0.2)
project(hsr123)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_add_env_hooks(hsr123 SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
