cmake_minimum_required(VERSION 2.8.3)
project(arc_tools)


find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
  	roscpp
    geometry_msgs
    sensor_msgs
    nav_msgs
    std_msgs
    eigen_catkin
    catkin_simple REQUIRED
    tf
)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
    src/coordinate_transform.cpp
    src/timing.cpp
    src/tf_kit.cpp
)

cs_install()
cs_export()
