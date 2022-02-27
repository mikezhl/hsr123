cmake_minimum_required(VERSION 3.0.2)
project(hsr123)

find_package(catkin REQUIRED COMPONENTS
  hsr_meshes
)

catkin_package(CATKIN_DEPENDS )

catkin_install_python(
  PROGRAMS
    src/increment_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_add_env_hooks(hsr123 SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
