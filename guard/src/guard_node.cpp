#include "../include/guard/guard.hpp"
bool status=1;
int main(int argc, char **argv)
{
  	ros::init(argc, argv, "high_level_guard");
	ros::NodeHandle node;
	//Guard used ?
	bool use_guard = true;
	if(strlen(*(argv + 1)) == 5) use_guard = false;
	//Create object.
	guard guard_object(node, use_guard);
	ros::spin();
 	return 0;
}
