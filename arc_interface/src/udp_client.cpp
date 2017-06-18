#include <arpa/inet.h>
#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>

#include <arc_msgs/State.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

//Define Constants.
const int BUFLEN = 512;
float MAX_STEERING_ANGLE;
float MAX_VELOCITY;
int MY_PORT;
int NI_PORT;
int QUEUE_LENGTH;
std::string NOTSTOP_TOPIC;
std::string REAR_LEFT_TOPIC;
std::string REAR_RIGHT_TOPIC;
std::string STELLGROESSEN_TOPIC;
std::string STEERING_CURRENT_TOPIC;
std::string VCU_CONTROLLER_STATE_TOPIC;
std::string VCU_LAUNCHING_COMMAND_TOPIC;
std::string VCU_PARAMETER_MODE_TOPIC;
std::string VCU_WORKING_INTERFACE_TOPIC;

//Network.
struct sockaddr_in si_me, si_other, si_NI;
int sock;
socklen_t slen;
//Notstop.
bool SEND_NOTSTOP = true;
//Rosbag play.
bool rosbag_play = true;
//Subscriber and publisher.
ros::Publisher rear_left_pub;
ros::Publisher rear_right_pub;
ros::Publisher steering_current_pub;
ros::Publisher vcu_controller_state_pub;
ros::Publisher vcu_working_pub;
ros::Subscriber notstop_sub;
ros::Subscriber stellgroessen_should_sub;
ros::Subscriber vcu_launching_sub;
ros::Subscriber vcu_parameter_sub;
ros::Subscriber vcu_ping_sub;
//Declaration of functions.
std::string convertCharArrayToString(char array[BUFLEN], int size);
std::string convertDoubleToString(double value);
double getValueFromMsg(std::string msg);
void handleReceivedMsg(std::string msg);
void notstopCallback(const std_msgs::Bool::ConstPtr& msg);
void printErrorAndFinish(std::string reason);
void setUpNetwork();
void setUpRosInterface(ros::NodeHandle* node);
void stellgroessenCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg);
void vcuLaunchingCallback(const std_msgs::Bool::ConstPtr& msg);
void vcuParameterModeCallback(const std_msgs::Float64::ConstPtr& msg);
void vcuPingCallback(const std_msgs::Float64::ConstPtr& msg);

int main(int argc, char** argv){
  //Init ros.
  ros::init(argc, argv, "arc_interface");
  ros::NodeHandle node;
  //Check rosbag.
  if(strlen(*(argv + 1)) == 5) rosbag_play = false;
  //Get constants from yaml file.
  node.getParam("/erod/MAX_STEERING_ANGLE", MAX_STEERING_ANGLE);
  node.getParam("/safety/MAX_ABSOLUTE_VELOCITY", MAX_VELOCITY);
  node.getParam("/general/MY_PORT", MY_PORT);
  node.getParam("/general/NI_PORT", NI_PORT);
  node.getParam("/general/QUEUE_LENGTH", QUEUE_LENGTH);
  node.getParam("/topic/NOTSTOP", NOTSTOP_TOPIC);
  node.getParam("/topic/STELLGROESSEN_SAFE", STELLGROESSEN_TOPIC);
  node.getParam("/topic/STATE_STEERING_ANGLE", STEERING_CURRENT_TOPIC);
  node.getParam("/topic/VCU_LAUNCHING_COMMAND", VCU_LAUNCHING_COMMAND_TOPIC);
  node.getParam("/topic/VCU_PARAMETER_MODE", VCU_PARAMETER_MODE_TOPIC);
  node.getParam("/topic/VCU_WORKING_INTERFACE", VCU_WORKING_INTERFACE_TOPIC);
  node.getParam("/topic/VCU_CONTROLLER_STATE", VCU_CONTROLLER_STATE_TOPIC);
  node.getParam("/topic/WHEEL_REAR_LEFT", REAR_LEFT_TOPIC);
  node.getParam("/topic/WHEEL_REAR_RIGHT", REAR_RIGHT_TOPIC);
  //Initialise addresses.
  setUpNetwork();
  if ((sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) printErrorAndFinish("socket");
  if(bind(sock, (struct sockaddr*)&si_me, sizeof(si_me)) == -1) printErrorAndFinish("binding");
  //Initialise ros interface.
  setUpRosInterface(&node);
  //Controlling sending rate.
  //Listening for and sending data.
  while(ros::ok()){
    //Receiving.
    int recv_len;
    char buffer_in[BUFLEN];
    if ((recv_len = recvfrom(sock, buffer_in, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == -1) 
           printErrorAndFinish("receiving");
    std::string buffer_in_string = convertCharArrayToString(buffer_in, recv_len);
    handleReceivedMsg(buffer_in_string);
    //Sending.
    ros::spinOnce();
  }
  //Reseting to manuell mode.
  std::string reset_string;
  reset_string = "am:" + convertDoubleToString(0.0);
  const char *buffer_out = reset_string.c_str();
  if (sendto(sock, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_NI, slen) == -1) 
        printErrorAndFinish("sending reseting");
  return 0;
}

std::string convertCharArrayToString(char array[BUFLEN], int size){
  std::string str;
  for (int i = 0; i < size; ++i){
    str += array[i];
  }
  return str;
}

std::string convertDoubleToString(double value){
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

double getValueFromMsg(std::string msg){
  //Get value string (beginning behind : ).
  std::string value_string(msg, 3, msg.length()-1);
  //Convert to number.
  const char *buffer = value_string.c_str();
  double value = atof(buffer);
  return value;
}

void handleReceivedMsg(std::string msg){
  //Get kind of msg and value.
  std::string kind = msg.substr(0,2);
  double value = getValueFromMsg(msg);
  //Creating ros msg.
  std_msgs::Float64 ros_msg;
  ros_msg.data = value;
  //Answers.
  if(kind == "si"){
    std_msgs::Float64 steering_msg;
    steering_msg.data = (value-1000)*M_PI/180;
    if(!rosbag_play) steering_current_pub.publish(steering_msg);
  } 
  else if(kind == "rl"){ 
    if(!rosbag_play) rear_left_pub.publish(ros_msg); 
  }
  else if(kind == "rr"){
    if(!rosbag_play) rear_right_pub.publish(ros_msg); 
  }
  else if(kind == "am") vcu_controller_state_pub.publish(ros_msg);
  else if(kind == "cc") vcu_working_pub.publish(ros_msg);
  else if(kind == "hn") {SEND_NOTSTOP = false; std::cout << "End: " << ros::Time::now() << std::endl;}
  else std::cout<<"ARC INTERFACE: Cannot assign msg " << msg << std::endl;
 }

void notstopCallback(const std_msgs::Bool::ConstPtr& msg){
  if(msg->data){
    std::string notstop_string = "hr:1";
    const char *buffer_out = notstop_string.c_str();
    if (sendto(sock, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_NI, slen) == -1) 
          printErrorAndFinish("sending notstop");
  }
}

void printErrorAndFinish(std::string reason){
  std::cout << "ARC_INTERFACE: Error due to " << reason << std::endl;
  //Finish ros and program.
  ros::shutdown();
  exit(0);
  ros::waitForShutdown();
}

void setUpNetwork(){
  //Computer (my) address.
  memset((char *) &si_me, 0, sizeof(si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(MY_PORT);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  //NI (destination) address.
  si_NI.sin_family = AF_INET;
  si_NI.sin_port = htons(NI_PORT);
  si_NI.sin_addr.s_addr = inet_addr("10.0.0.8");
  //Socket length.
  slen = sizeof(si_other);
}

void setUpRosInterface(ros::NodeHandle* node){
  //Publisher and subscriber
  rear_left_pub = node->advertise<std_msgs::Float64>(REAR_LEFT_TOPIC, QUEUE_LENGTH);
  rear_right_pub = node->advertise<std_msgs::Float64>(REAR_RIGHT_TOPIC, QUEUE_LENGTH);
  steering_current_pub = node->advertise<std_msgs::Float64>(STEERING_CURRENT_TOPIC, QUEUE_LENGTH);
  vcu_controller_state_pub = node->advertise<std_msgs::Float64>(VCU_CONTROLLER_STATE_TOPIC, QUEUE_LENGTH);
  vcu_working_pub = node->advertise<std_msgs::Float64>(VCU_WORKING_INTERFACE_TOPIC, QUEUE_LENGTH);
  notstop_sub = node->subscribe(NOTSTOP_TOPIC, QUEUE_LENGTH, notstopCallback);
  stellgroessen_should_sub = node->subscribe(STELLGROESSEN_TOPIC, QUEUE_LENGTH, stellgroessenCallback);
  vcu_launching_sub = node->subscribe(VCU_LAUNCHING_COMMAND_TOPIC, QUEUE_LENGTH, vcuLaunchingCallback);
  vcu_parameter_sub = node->subscribe(VCU_PARAMETER_MODE_TOPIC, QUEUE_LENGTH, vcuParameterModeCallback);
  vcu_ping_sub = node->subscribe(VCU_WORKING_INTERFACE_TOPIC, QUEUE_LENGTH, vcuPingCallback);
}

void stellgroessenCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg){
  //Sending should velocity [m/s].
  double vel_should = msg->speed;
  if(vel_should > 0.7*MAX_VELOCITY) vel_should = 0.7*MAX_VELOCITY;
  std::string vel_string = "vs:" + convertDoubleToString(vel_should);
  const char *buffer_out_vel = vel_string.c_str();
  if (sendto(sock, buffer_out_vel, sizeof(buffer_out_vel), 0, (struct sockaddr*) &si_NI, slen) == -1) 
        printErrorAndFinish("sending velocity_should"); 
  //Sending should steering angle [deg, shifted]. 
  double steering_should = (msg->steering_angle)/M_PI*180;
  if(steering_should < - MAX_STEERING_ANGLE) steering_should = - MAX_STEERING_ANGLE;
  if(steering_should > + MAX_STEERING_ANGLE) steering_should = + MAX_STEERING_ANGLE;
  std::string steering_string = "ss:" + convertDoubleToString(steering_should);
  const char *buffer_out_steering = steering_string.c_str();
  if (sendto(sock, buffer_out_steering, sizeof(buffer_out_steering), 0, (struct sockaddr*) &si_NI, slen) == -1) 
        printErrorAndFinish("sending steering_should");     
}

void vcuLaunchingCallback(const std_msgs::Bool::ConstPtr& msg){
  //Start Controller if GUI button pushed and system ready.
  std::string vcu_launching_string;
  if(!msg->data) vcu_launching_string = "am:" + convertDoubleToString(0.0);
  if(msg->data) vcu_launching_string = "am:" + convertDoubleToString(1.0);
  const char *buffer_out = vcu_launching_string.c_str();
  if (sendto(sock, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_NI, slen) == -1) 
        printErrorAndFinish("sending vcu launching");
}

void vcuParameterModeCallback(const std_msgs::Float64::ConstPtr& msg){
  //Getting mode.
  double vcu_mode = 1.0;
  //double vcu_mode = msg->data;
  //Sending to NI.
  std::string vcu_mode_string = "ec:" + convertDoubleToString(vcu_mode);
  const char *buffer_out = vcu_mode_string.c_str();
  if (sendto(sock, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_NI, slen) == -1) 
        printErrorAndFinish("sending vcu parameter mode");
}

void vcuPingCallback(const std_msgs::Float64::ConstPtr& msg){
  //Getting ping.
  double ping = msg->data;
  //Sending to NI.
  std::string vcu_ping_string = "cc:" + convertDoubleToString(ping);
  const char *buffer_out = vcu_ping_string.c_str();
  if (sendto(sock, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_NI, slen) == -1) 
        printErrorAndFinish("sending vcu ping");
}
