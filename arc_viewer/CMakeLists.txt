cmake_minimum_required(VERSION 2.8.9)
project(arc_viewer)

find_package(catkin REQUIRED COMPONENTS roscpp arc_msgs arc_tools)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs sensor_msgs ackermann_msgs std_msgs arc_msgs arc_tools
  DEPENDS system-lib
)

qt5_add_resources(QT_RESOURCES_CPP qt_resources/stop.qrc qt_resources/arc.qrc)
qt5_wrap_cpp(QT_MOC_HPP src/ControlWindow.hpp
						src/RosInterface.hpp)

include_directories(src ${catkin_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_executable(arc_gui	   src/gui.cpp
						   src/ControlWindow.cpp
						   src/RosInterface.cpp
						   ${QT_RESOURCES_CPP}
              			   ${QT_MOC_HPP})

target_link_libraries(arc_gui ${QT_LIBRARIES}
            		  ${catkin_LIBRARIES} Qt5::Widgets)

install(TARGETS arc_gui
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(ps_control src/ps_control.cpp)
target_link_libraries(ps_control ${catkin_LIBRARIES})