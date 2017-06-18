#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <stdlib.h>
#include <string>

#include <arc_msgs/State.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16MultiArray.h>

//Constants.
bool INIT_MODE = true;
bool BUTTON_START = true;
std::string PATH_NAME;
std::string PATH_LENGTH;
int QUEUE_LENGTH;
std::string VCU_PARAMETER_MODE;
std::string GPS_TOPIC;
std::string GUARD_TOPIC;
std::string LAUNCHING_INFO_TOPIC;
std::string OBSTACLE_DETECTION_TOPIC;
std::string ORB_SLAM_TOPIC;
std::string PURE_PURSUIT_TOPIC; 
std::string READY_FOR_DRIVING_TOPIC;
std::string READY_FOR_LAUNCHING_TOPIC;
std::string ROVIO_TOPIC;
std::string STATE_ESTIMATION_TOPIC;
std::string STELLGROESSEN_TOPIC;
std::string VELODYNE_TOPIC;
std::string VCU_LAUNCHING_TOPIC;
std::string VCU_PARAMETER_MODE_TOPIC;
std::string VCU_WORKING_INTERFACE_TOPIC;
std::string VI_TOPIC;
//Sensor or node used.
bool USE_CONTROLLING = true;
bool USE_GPS = true;
bool USE_NI_CLIENT = true;
bool USE_OBSTACLE_DETECTION = true;
bool USE_GUARD = true;
bool USE_STATE_ESTIMATION = true;
bool USE_VELODYNE = true;
bool USE_VI = true;
//Sensor or node initialised.
bool ALL_INITIALISED = false;
bool READY_CONTROL = false;
bool READY_GPS = false;
bool READY_GUARD = false;
bool READY_NI_CLIENT = false;
bool READY_OBSTACLE_DETECTION = false;
bool READY_ORBSLAM = false;
bool READY_ROVIO = false;
bool READY_STATE_ESTIMATION = false;
bool READY_VELODYNE = false;
bool READY_VI = false;
//Subscriber and Publisher.
ros::Subscriber gps_sub;
ros::Subscriber guard_sub;
ros::Subscriber obstacle_detection_sub;
ros::Subscriber pure_pursuit_sub;
ros::Subscriber orb_slam_sub;
ros::Subscriber rovio_sub;
ros::Subscriber state_estimation_sub;
ros::Subscriber velodyne_sub;
ros::Subscriber vcu_sub;
ros::Subscriber vi_sub;
ros::Publisher launching_info_pub;
ros::Publisher ready_driving_pub;
ros::Publisher ready_launching_pub;
ros::Publisher stellgroessen_reset_pub;
ros::Publisher vcu_launching_pub;
ros::Publisher vcu_mode_pub;
ros::Publisher vcu_ping_pub;
//Declaration of functions.
void checkingAllInitialised();
void copyPathFile(std::string original);
void gpsCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
void guardCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg);
void initPrepartion(ros::NodeHandle* node);
void obstacleCallback(const std_msgs::Bool::ConstPtr& msg);
void orbslamCallback(const nav_msgs::Odometry::ConstPtr& msg);
void purePursuitCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg);
void rovioCallback(const nav_msgs::Odometry::ConstPtr& msg);
void shortenPathFile(std::string original, std::string path_length);
void stateCallback(const arc_msgs::State::ConstPtr& msg);
void vcuCallback(const std_msgs::Float64::ConstPtr& msg);
void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void viCallback(const sensor_msgs::Image::ConstPtr& msg);

int main(int argc, char** argv){
	//Init ROS.
	ros::init(argc, argv, "preparation");
	ros::NodeHandle node;
	//Getting parameter.
	PATH_NAME = *(argv + 1);
	if(strlen(*(argv + 2)) == 5) INIT_MODE = false;
	PATH_LENGTH = *(argv + 3);
	if(strlen(*(argv + 4)) == 5) USE_GPS = false;
	if(strlen(*(argv + 5)) == 5) USE_VI = false;
	if(strlen(*(argv + 6)) == 5) USE_VELODYNE = false;
	if(strlen(*(argv + 7)) == 5) USE_NI_CLIENT = false;
	if(strlen(*(argv + 8)) == 5) USE_STATE_ESTIMATION = false;
	if(strlen(*(argv + 9)) == 5) USE_OBSTACLE_DETECTION = false;
	if(strlen(*(argv + 10)) == 5) USE_GUARD = false;
	if(strlen(*(argv + 11)) == 5) USE_CONTROLLING = false;
	if(strlen(*(argv + 12)) == 5) BUTTON_START = false;
	//Initialise.
	initPrepartion(&node);
	//Edit path file and starting pure pursuit.
	if(PATH_LENGTH != "full" && INIT_MODE) shortenPathFile(PATH_NAME, PATH_LENGTH);
	if(PATH_LENGTH == "full" && INIT_MODE) copyPathFile(PATH_NAME);
	std_msgs::Bool ready_for_launching_msg;
	ready_for_launching_msg.data = true;
	ready_launching_pub.publish(ready_for_launching_msg);
	//Reset steering and velocity (ss:10).
	ackermann_msgs::AckermannDrive reset_msg;
	reset_msg.steering_angle = -900*180/M_PI;
	reset_msg.speed = 0.0;
	stellgroessen_reset_pub.publish(reset_msg);
	//Preparing gui information.
	std_msgs::Int16MultiArray launching_info_msg;
	launching_info_msg.data.clear();
	for(int i=0;i<10;i++) launching_info_msg.data.push_back(0);
	ros::Rate loop_rate(1);
	//Checking nodes runnning and publishing ready topic.
	while(!ALL_INITIALISED){
		//Ping VCU.
		if(!READY_NI_CLIENT){
			std_msgs::Float64 vcu_ping_msg;
			//Ping VCU: Connection working, calibrate and activate autonomous steering.
			if(INIT_MODE) vcu_ping_msg.data = 5.0;
			else vcu_ping_msg.data = 0.0;
			vcu_ping_pub.publish(vcu_ping_msg);
			std_msgs::Float64 vcu_mode_msg;
			vcu_mode_msg.data = 1.0;
			// if(VCU_PARAMETER_MODE == "street") vcu_mode_msg.data = 1.0;
			// if(VCU_PARAMETER_MODE == "lift") vcu_mode_msg.data = 0.0;
			vcu_mode_pub.publish(vcu_mode_msg);
		}
		//Publishing launch information.
		launching_info_msg.data[0] = READY_CONTROL;
		launching_info_msg.data[1] = READY_GPS;
		launching_info_msg.data[2] = READY_NI_CLIENT;
		launching_info_msg.data[3] = READY_OBSTACLE_DETECTION;
		launching_info_msg.data[4] = READY_GUARD;
		launching_info_msg.data[5] = READY_ROVIO;
		launching_info_msg.data[6] = READY_STATE_ESTIMATION;
		launching_info_msg.data[7] = READY_ORBSLAM;
		launching_info_msg.data[8] = READY_VELODYNE;
		launching_info_msg.data[9] = READY_VI;
		launching_info_pub.publish(launching_info_msg);
		//Checking if nodes running.
		checkingAllInitialised();
		//Controlling loop.
		ros::spinOnce();
		loop_rate.sleep();
	}
	std_msgs::Bool ready_for_driving_msg;
	ready_for_driving_msg.data = true;
	ready_driving_pub.publish(ready_for_driving_msg);
	std::cout << std::endl <<"PREPARATION: Everything initialised, ready to go !" << std::endl;
	//Starting automatically iff button start is not set.
	if(!BUTTON_START && INIT_MODE){
		std_msgs::Bool launching_msg;
        launching_msg.data = true;
        vcu_launching_pub.publish(launching_msg);
	}
	return 0;
}

void checkingAllInitialised(){
	if(USE_CONTROLLING && !READY_CONTROL && INIT_MODE) ALL_INITIALISED = false;
	else if(USE_GPS && !READY_GPS) ALL_INITIALISED = false;
	else if(USE_GUARD && !READY_GUARD && INIT_MODE) ALL_INITIALISED = false;
	else if(USE_NI_CLIENT && !READY_NI_CLIENT) ALL_INITIALISED = false;
	else if(USE_OBSTACLE_DETECTION && !READY_OBSTACLE_DETECTION && INIT_MODE) ALL_INITIALISED = false;
	else if(USE_STATE_ESTIMATION && !READY_STATE_ESTIMATION) ALL_INITIALISED = false;
	else if(USE_STATE_ESTIMATION && !READY_ORBSLAM) ALL_INITIALISED = false;
	else if(USE_STATE_ESTIMATION && !READY_ROVIO) ALL_INITIALISED = false;
	else if(USE_VELODYNE && !READY_VELODYNE) ALL_INITIALISED = false;
	else if(USE_VI && !READY_VI) ALL_INITIALISED = false;
	else ALL_INITIALISED = true;
}

void copyPathFile(std::string original){
	//Open original file.
	std::string original_filename = original + "_teach.txt";
	std::ifstream in_file(original_filename.c_str());
	//Open edit file.
	std::string edited_filename = original + "_teach_edited.txt";
    std::ofstream out_file(edited_filename.c_str());
    //Copy file.
    out_file << in_file.rdbuf();	
}

void gpsCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
	if(!READY_GPS) std::cout << "PREPARATION: GPS initialised" << std::endl;
	READY_GPS = true;
}

void guardCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg){
	if(!READY_GUARD) std::cout << "PREPARATION: Guard initialised" << std::endl;
	READY_GUARD = true;
}

void initPrepartion(ros::NodeHandle* node){
	//Getting ros Parameter.
	node->getParam("/general/QUEUE_LENGTH", QUEUE_LENGTH);
	node->getParam("/general/VCU_PARAMETER_MODE", VCU_PARAMETER_MODE);
	node->getParam("/topic/GPS_POSITION", GPS_TOPIC);
	node->getParam("/topic/LASER_STOP", OBSTACLE_DETECTION_TOPIC);	
	node->getParam("/topic/ORB_SLAM_ODOMETRY", ORB_SLAM_TOPIC);
	node->getParam("/topic/READY_FOR_DRIVING", READY_FOR_DRIVING_TOPIC);
	node->getParam("/topic/READY_FOR_LAUNCHING", READY_FOR_LAUNCHING_TOPIC);
	node->getParam("/topic/ROVIO_ODOMETRY", ROVIO_TOPIC);
	node->getParam("/topic/RUNNING_PROGRAMMES", LAUNCHING_INFO_TOPIC);
	node->getParam("/topic/STATE", STATE_ESTIMATION_TOPIC);
	node->getParam("/topic/STELLGROESSEN", PURE_PURSUIT_TOPIC);
	node->getParam("/topic/STELLGROESSEN_SAFE", GUARD_TOPIC);
	node->getParam("/topic/STELLGROESSEN_SAFE", STELLGROESSEN_TOPIC);
	node->getParam("/topic/VCU_LAUNCHING_COMMAND", VCU_LAUNCHING_TOPIC);
	node->getParam("/topic/VCU_PARAMETER_MODE", VCU_PARAMETER_MODE_TOPIC);
	node->getParam("/topic/VCU_WORKING_INTERFACE", VCU_WORKING_INTERFACE_TOPIC);
	node->getParam("/topic/VELODYNE_POINTCLOUD", VELODYNE_TOPIC);
	node->getParam("/topic/VI_CAMERA_LEFT", VI_TOPIC);
	//Setting publisher and subcriber.
	launching_info_pub = node->advertise<std_msgs::Int16MultiArray>(LAUNCHING_INFO_TOPIC, QUEUE_LENGTH);
	ready_driving_pub = node->advertise<std_msgs::Bool>(READY_FOR_DRIVING_TOPIC, QUEUE_LENGTH);
	ready_launching_pub = node->advertise<std_msgs::Bool>(READY_FOR_LAUNCHING_TOPIC, QUEUE_LENGTH);
	stellgroessen_reset_pub = node->advertise<ackermann_msgs::AckermannDrive>(STELLGROESSEN_TOPIC, QUEUE_LENGTH);
	vcu_launching_pub = node->advertise<std_msgs::Bool>(VCU_LAUNCHING_TOPIC, QUEUE_LENGTH);
	vcu_mode_pub = node->advertise<std_msgs::Float64>(VCU_PARAMETER_MODE_TOPIC, QUEUE_LENGTH);
	vcu_ping_pub = node->advertise<std_msgs::Float64>(VCU_WORKING_INTERFACE_TOPIC, QUEUE_LENGTH);
	if(USE_CONTROLLING) pure_pursuit_sub = node->subscribe(PURE_PURSUIT_TOPIC, QUEUE_LENGTH, purePursuitCallback);
	if(USE_GPS) gps_sub = node->subscribe(GPS_TOPIC, QUEUE_LENGTH, gpsCallback);
	if(USE_GUARD) guard_sub = node->subscribe(GUARD_TOPIC, QUEUE_LENGTH, guardCallback);
	if(USE_NI_CLIENT) vcu_sub = node->subscribe(VCU_WORKING_INTERFACE_TOPIC, QUEUE_LENGTH, vcuCallback);
	if(USE_OBSTACLE_DETECTION) obstacle_detection_sub = node->subscribe(OBSTACLE_DETECTION_TOPIC, QUEUE_LENGTH, obstacleCallback);
	if(USE_STATE_ESTIMATION){
		state_estimation_sub = node->subscribe(STATE_ESTIMATION_TOPIC, QUEUE_LENGTH, stateCallback);
		orb_slam_sub = node->subscribe(ORB_SLAM_TOPIC, QUEUE_LENGTH, orbslamCallback);
		rovio_sub = node->subscribe(ROVIO_TOPIC, QUEUE_LENGTH, rovioCallback);	
	} 
	if(USE_VELODYNE) velodyne_sub = node->subscribe(VELODYNE_TOPIC, QUEUE_LENGTH, velodyneCallback);
	if(USE_VI) vi_sub = node->subscribe(VI_TOPIC, QUEUE_LENGTH, viCallback);
}

void vcuCallback(const std_msgs::Float64::ConstPtr& msg){
	if(msg->data == 1.0){
		if(!READY_NI_CLIENT) std::cout << "PREPARATION: Interface initialised" << std::endl;
		READY_NI_CLIENT = true;
	}
}

void obstacleCallback(const std_msgs::Bool::ConstPtr& msg){
	if(!READY_OBSTACLE_DETECTION) std::cout << "PREPARATION: Obstacle detection initialised" << std::endl;
	READY_OBSTACLE_DETECTION = true;
}

void orbslamCallback(const nav_msgs::Odometry::ConstPtr& msg){
	double position_value = msg->pose.pose.position.x*msg->pose.pose.position.x + msg->pose.pose.position.y*msg->pose.pose.position.y
							+ msg->pose.pose.position.z*msg->pose.pose.position.z;
	if(INIT_MODE && position_value > 0.0){
		if(!READY_ORBSLAM) std::cout << "PREPARATION: RSLAM initialised" << std::endl;
		READY_ORBSLAM = true;
		return;
	}
	if(!INIT_MODE){
		if(!READY_ORBSLAM) std::cout << "PREPARATION: RSLAM initialised" << std::endl;
		READY_ORBSLAM = true;
	}	
}

void purePursuitCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg){
	if(!READY_CONTROL) std::cout << "PREPARATION: Pure Pursuit initialised" << std::endl; 
	READY_CONTROL = true;
}

void rovioCallback(const nav_msgs::Odometry::ConstPtr& msg){
	if(!READY_ROVIO) std::cout << "PREPARATION: Rovio initialised" << std::endl;
	READY_ROVIO = true;
}

void shortenPathFile(std::string original, std::string path_length_string){
	//Get path length from string.
	char buffer[20];
	for (int i=0; i<path_length_string.size(); ++i){
    	buffer[i] = path_length_string[i];
  	}
  	double path_length = atof(buffer);
	//Read original file.
	std::fstream fin;
	std::string original_filename = original + "_teach.txt";
	fin.open(original_filename.c_str());
	if(!fin.is_open()){
	   std::cout<<std::endl<<"PREPARATION: Error with opening of  "
	            <<original_filename<<std::endl;
	}
	//Getting original file length.
  	fin.seekg (-2, fin.end); 
  	int file_length = fin.tellg();
  	fin.seekg (0, fin.beg);
	//Creating char array containing the original file.
	char * file = new char [file_length+1];
	fin.read(file,file_length);
	std::istringstream in_stream(file,std::ios::in);
	delete[] file;  
	fin.close ();
	//Open output file.
	std::ofstream fout;
	std::string edited_filename = original + "_teach_edited.txt";
	fout.open(edited_filename.c_str());
	if(!fout.is_open()){
	   std::cout<<std::endl<<"PREPARATION: Error with opening of  "
	            <<edited_filename<<std::endl;
	}
	//Writing into new file until max distance is reached.
	int i = 0;
	int array_index;  
	Eigen::Vector3d position(0,0,0);
	Eigen::Vector3d last_position(0,0,0);
	Eigen::Vector4d quat;
	double velocity;
	bool stop;
	double path_distance = 0.0;
	while(position.norm()<path_length && !in_stream.eof()){
	  //Read.
	  in_stream>>array_index;
	  in_stream>>position(0);
	  in_stream>>position(1);
	  in_stream>>position(2);
	  in_stream>>quat(0);
	  in_stream>>quat(1);
	  in_stream>>quat(2);
	  in_stream>>quat(3);
	  in_stream>>velocity;
	  in_stream>>stop;
	  in_stream.ignore (300, '|');
	  //Write.
	  fout<<array_index<<" "<<
           position(0)<<" "<<position(1)<<" "<<position(2)<<" "<<
           quat(0)<<" "<<quat(1)<<" "<<quat(2)<<" "<<quat(3)<<" "<<
           velocity <<" "<<stop<<"|";
      //Indexing.
      i++;
      path_distance += (last_position-position).norm();
      last_position = position;
	}
	fout.close();
}

void stateCallback(const arc_msgs::State::ConstPtr& msg){
	if(!READY_STATE_ESTIMATION) std::cout << "PREPARATION: State Estimation initialised" << std::endl;
	READY_STATE_ESTIMATION = true;
}

void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
	if(!READY_VELODYNE) std::cout << "PREPARATION: Velodyne initialised" << std::endl;
	READY_VELODYNE = true;
}	

void viCallback(const sensor_msgs::Image::ConstPtr& msg){
	if(!READY_VI) std::cout << "PREPARATION: VI initialised" << std::endl;
	READY_VI = true;
}
