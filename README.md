# arc_final

Final code of project ARC, used to climb the Swiss Klausenpass autonomously with the Teach and Repeat method, including a state estimation,. control and obstacle detection software as well as an VCU interface.

Tested on Ubuntu 14.04 and utilising the ROS-Framework.



## Installation

Use following commands on the terminal before building the package:

wstool init

wstool merge arc_final/dependencies.rosinstall 

wstool update

For velodyne:

sudo apt-get install libpcap-dev

For arc_viewer:

sudo apt-get install qt5-default

For ps3-driver:
sudo apt-get install libusb-dev

##Using
roslaunch arc_launch arc.launch Name:=(Desired path to created files) Mode:=(false [Teach] or true [Repeat])


