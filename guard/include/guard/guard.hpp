#include "ackermann_msgs/AckermannDrive.h"
#include "arc_msgs/State.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

class guard{
public:
	//Konstruktor.
	guard(const ros::NodeHandle &nh, bool use_guard);
	//Save the incoming current state.
	void getCommand (const ackermann_msgs::AckermannDrive::ConstPtr& save_com);
	//Checking stop bools.
	void getGuiStop (const std_msgs::Bool::ConstPtr& msg);
	void getLaserStop (const std_msgs::Bool::ConstPtr& msg);
	void getState (const arc_msgs::State::ConstPtr& arc_state);
	void getVcuStop (const std_msgs::Float64::ConstPtr& msg);
	void chooseAndPublish();
private:
	ros::NodeHandle nh_;
	//Guard used ?
	bool use_guard_;
	//Subscriber and Publisher.
	ros::Publisher safe_command_pub_;
	ros::Publisher not_stop_pub_;
	ros::Subscriber gui_stop_sub_;
	ros::Subscriber laser_stop_sub_;
	ros::Subscriber state_sub_;
	ros::Subscriber stellgroessen_sub_;
	ros::Subscriber vcu_stop_sub_;
	//Transfer varible.
	ackermann_msgs::AckermannDrive guard_stellgroessen_;
	//Status variable. 
	bool gui_stop_;
	bool laser_stop_;
	bool state_stop_;
	bool vcu_stop_;
	bool not_stop_;
};

	


	
