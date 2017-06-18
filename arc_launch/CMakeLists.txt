cmake_minimum_required(VERSION 2.8.3)
project(arc_launch)


find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
  	roscpp
  	eigen_catkin
  	std_msgs
  	sensor_msgs
  	nav_msgs
  	geometry_msgs
    catkin_simple REQUIRED
)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(preparation src/preparation.cpp)
target_link_libraries(preparation ${catkin_LIBRARIES})

cs_install()
cs_export()
