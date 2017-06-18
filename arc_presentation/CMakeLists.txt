cmake_minimum_required(VERSION 2.8.3)
project(arc_presentation)

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  ackermann_msgs
  pcl_conversions
  pcl_ros
  catkin_simple REQUIRED
)

catkin_simple(ALL_DEPS_REQUIRED)

add_executable(lift_presentation src/lift_presentation.cpp)
target_link_libraries(lift_presentation ${catkin_LIBRARIES})

add_executable(lift_presentation_laser src/lift_presentation_laser.cpp)
target_link_libraries(lift_presentation_laser ${catkin_LIBRARIES})

cs_install()
cs_export()

