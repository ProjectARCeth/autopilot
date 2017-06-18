#include "arc_msgs/State.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <iostream>
#include <fstream>
#include <sstream>
class gridAnalyser
{
public:
	//Constructor.
	gridAnalyser(const ros::NodeHandle &nh);
	//Destructor.
	~gridAnalyser();
	//Function which saves the incoming state.
	void getState (const arc_msgs::State::ConstPtr& arc_state);
	//Function which saves the incoming occupancy map.
	void getGridMap (const nav_msgs::OccupancyGrid::ConstPtr& grip_map);
	//Function to save from TXT-File to a Path copied from Moritz.
	void readPathFromTxt(std::string inFileName);
	//Function which produces inflated map.
	void createSafeMap(const nav_msgs::OccupancyGrid::ConstPtr& grip_map);
	//Funtion which calculates the n in row-major order from the cell index in (x, y).
	int calculateIndex (int x, int y);
	//Function which inflates the obstacle around point (x, y).
	void inflate(int x, int y);
private:
	//Ros-Constants:.
	//NodeHandle.
	ros::NodeHandle nh_;
	//Publisher for the boolean.
	ros::Publisher status_pub;
	//Subscriber to the state.
	ros::Subscriber state_sub;
	//Subscriber to the incoming map.
	ros::Subscriber grid_map_sub;
	//Variable to store the path.
	nav_msgs::Path path_;
	//Variable to store number of cells in direction of travel (y).
	int height_;
	//variable to store the number of cells orthogonal to direction of travel (x). 
	int width_;
	//Variable to store the resolution. 
	float resolution_;
	//Variable to store the safe map with inflation.
	nav_msgs::OccupancyGrid	grid_safe_;
	//Bool to store the status (safe/unsafe). 
	bool status_;
	//Half of width of the erod.
	float erod_width_;
	//Factor of safety.
	float FoS_;
	
};
