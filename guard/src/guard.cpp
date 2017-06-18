#include "../include/guard/guard.hpp"

std::string GUI_STOP_TOPIC;
std::string LASER_STOP_TOPIC;
std::string NOTSTOP_TOPIC;
std::string STATE_TOPIC;
std::string STELLGROESSEN_TOPIC;
std::string STELLGROESSEN_SAFE_TOPIC;
std::string STOP_NI_TOPIC;
int QUEUE_LENGTH;

guard::guard(const ros::NodeHandle &nh, bool use_guard) : nh_(nh), use_guard_(use_guard) {	
	//Initialisierung.
	gui_stop_ = 0;
	not_stop_ = 0;
	state_stop_ = 0;
	vcu_stop_ = 0 ;
	laser_stop_ = 0 ;
	//Getting parameters.
	nh_.getParam("/general/QUEUE_LENGTH", QUEUE_LENGTH);
	nh_.getParam("/topic/GUI_STOP", GUI_STOP_TOPIC);
	nh_.getParam("/topic/LASER_STOP", LASER_STOP_TOPIC);
	nh_.getParam("/topic/NOTSTOP", NOTSTOP_TOPIC);
	nh_.getParam("/topic/STATE", STATE_TOPIC);
	nh_.getParam("/topic/STELLGROESSEN", STELLGROESSEN_TOPIC);
	nh_.getParam("/topic/STELLGROESSEN_SAFE", STELLGROESSEN_SAFE_TOPIC);
	nh_.getParam("/topic/VCU_CONTROLLER_STATE", STOP_NI_TOPIC);	
	//Publishers and Subcriber.
	safe_command_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>(STELLGROESSEN_SAFE_TOPIC, QUEUE_LENGTH);
	not_stop_pub_ = nh_.advertise<std_msgs::Bool>(NOTSTOP_TOPIC, QUEUE_LENGTH);
	gui_stop_sub_ = nh_.subscribe(GUI_STOP_TOPIC, QUEUE_LENGTH, &guard::getGuiStop, this);
	laser_stop_sub_ = nh_.subscribe(LASER_STOP_TOPIC, QUEUE_LENGTH, &guard::getLaserStop, this);
	state_sub_ = nh_.subscribe(STATE_TOPIC, QUEUE_LENGTH, &guard::getState, this);
	stellgroessen_sub_ = nh_.subscribe(STELLGROESSEN_TOPIC, QUEUE_LENGTH, &guard::getCommand, this);
	vcu_stop_sub_ = nh_.subscribe(STOP_NI_TOPIC, QUEUE_LENGTH, &guard::getVcuStop, this);
}

void guard::chooseAndPublish(){
	if( (state_stop_ == 0) && (vcu_stop_ == 0) && (laser_stop_ == 0) && (gui_stop_ == 0) 
		|| use_guard_ == 0){
		not_stop_ = 0;
		std_msgs::Bool notstop_msg;
		notstop_msg.data = not_stop_;
		not_stop_pub_.publish(notstop_msg);
		safe_command_pub_.publish(guard_stellgroessen_);
	}
	else{ 
		not_stop_ = 1;
		guard_stellgroessen_.speed = 0;
		std::cout<<"!!!!!!!!!!NOTSTOP!!!!!!!!!!!!!!!"<<std::endl;
		std_msgs::Bool notstop_msg;
		notstop_msg.data = not_stop_;
		ros::Rate looprate(10);
		// while(ros::ok()){
		for(int i=0;i<15;++i){
			not_stop_pub_.publish(notstop_msg);
			safe_command_pub_.publish(guard_stellgroessen_);
			looprate.sleep();
		}
		// not_stop_pub_.publish(notstop_msg);
	}
}

void guard::getCommand (const ackermann_msgs::AckermannDrive::ConstPtr& msg){
	guard_stellgroessen_=*msg;
	chooseAndPublish();
}

void guard::getGuiStop(const std_msgs::Bool::ConstPtr& msg){
	gui_stop_ = msg->data;
	if(gui_stop_) std::cout << "NOTSTOP: GUI wants to stop !" << std::endl;
	chooseAndPublish();
}

void guard::getLaserStop(const std_msgs::Bool::ConstPtr& msg){
	laser_stop_=msg->data;
	if(laser_stop_) std::cout << "NOTSTOP: Laser wants to stop !" << std::endl;
	chooseAndPublish();
}

void guard::getState(const arc_msgs::State::ConstPtr& arc_state){	
	state_stop_=arc_state->stop;
	if(state_stop_) std::cout << "NOTSTOP: State estimation wants to stop !" << std::endl;
	chooseAndPublish();
}

void guard::getVcuStop(const std_msgs::Float64::ConstPtr& msg){
	if(msg->data == 99) vcu_stop_ = true;
	else vcu_stop_ = false;
	if(vcu_stop_) std::cout << "NOTSTOP: VCU wants to stop !" << std::endl;
	chooseAndPublish();
}

