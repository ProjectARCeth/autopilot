cmake_minimum_required(VERSION 2.8.3)
project(guard)

## Find catkin and any catkin packages
find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  catkin_simple
  arc_tools
    )


catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
src/guard.cpp

)


cs_add_executable(high_level_guard src/guard_node.cpp)
target_link_libraries(high_level_guard ${catkin_LIBRARIES} ${PROJECT_NAME})

cs_install()
cs_export()
