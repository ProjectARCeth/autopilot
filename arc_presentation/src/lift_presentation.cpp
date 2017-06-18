#include <arpa/inet.h>
#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <signal.h>

#include <ackermann_msgs/AckermannDrive.h>

//Define Constants.
const int BUFLEN = 512;
float MAX_STEERING_ANGLE;
float MAX_VELOCITY;
int MY_PORT;
int NI_PORT;
int QUEUE_LENGTH;
double time_start;

//Network.
struct sockaddr_in si_me, si_other, si_NI;
int sock;
socklen_t slen;
//Subscriber and publisher.
ros::Subscriber stellgroessen_should_sub;
//Declaration of functions.
std::string convertCharArrayToString(char array[BUFLEN], int size);
std::string convertDoubleToString(double value);
void printErrorAndFinish(std::string reason);
void setUpNetwork();
void setUpRosInterface(ros::NodeHandle* node);
void stellgroessenCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg);

int main(int argc, char** argv){
  //Init ros.
  ros::init(argc, argv, "arc_interface");
  ros::NodeHandle node;
  time_start = ros::Time::now().toSec();
  //Get constants from yaml file.
  node.getParam("/erod/MAX_STEERING_ANGLE", MAX_STEERING_ANGLE);
  node.getParam("/safety/MAX_ABSOLUTE_VELOCITY", MAX_VELOCITY);
  node.getParam("/general/MY_PORT", MY_PORT);
  node.getParam("/general/NI_PORT", NI_PORT);
  //Initialise addresses.
  setUpNetwork();
  if ((sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) printErrorAndFinish("socket");
  if(bind(sock, (struct sockaddr*)&si_me, sizeof(si_me)) == -1) printErrorAndFinish("binding");
  //Initialise ros interface.
  setUpRosInterface(&node);
  //Launching.
  std::string vcu_ping_string = "cc:" + convertDoubleToString(5.0);
  const char *buffer_out = vcu_ping_string.c_str();
  if (sendto(sock, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_NI, slen) == -1) 
        printErrorAndFinish("sending vcu ping");
  double vcu_mode = 0.0;
  std::string vcu_mode_string = "ec:" + convertDoubleToString(vcu_mode);
  buffer_out = vcu_mode_string.c_str();
  if (sendto(sock, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_NI, slen) == -1) 
        printErrorAndFinish("sending vcu parameter mode");
  std::string vcu_launching_string;
  vcu_launching_string = "am:" + convertDoubleToString(1.0);
  buffer_out = vcu_launching_string.c_str();
  if (sendto(sock, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_NI, slen) == -1) 
        printErrorAndFinish("sending vcu launching");
  //Listening for and sending data.
  while(ros::Time::now().toSec()-time_start<730/0.85 && ros::ok()){
    //Sending.
    ros::spinOnce();
  }
  //Reseting to manuell mode.
  std::string reset_string;
  reset_string = "am:" + convertDoubleToString(0.0);
  buffer_out = reset_string.c_str();
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

void printErrorAndFinish(std::string reason){
  //std::cout << "ARC_INTERFACE: Error due to " << reason << std::endl;
  //Finish ros and program.
  ros::shutdown();
  exit(0);
  ros::waitForShutdown();
}

void setUpRosInterface(ros::NodeHandle* node){
  stellgroessen_should_sub = node->subscribe("stellgroessen_safe", 10, stellgroessenCallback);
}

void stellgroessenCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg){
  //Sending should velocity [m/s].
  double vel_should = msg->speed;
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
