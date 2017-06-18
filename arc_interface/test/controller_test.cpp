#include <arpa/inet.h>
#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>

#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Float64.h>

//Define Constants.
const int BUFLEN = 512;
int MY_PORT = 8010;
int NI_PORT = 8001;
float MAX_STEERING_ANGLE;
std::string STELLGROESSEN_TOPIC;
std::string STEERING_CURRENT_TOPIC;
//Network.
struct sockaddr_in si_me, si_other, si_NI;
int sock;
socklen_t slen;
//Subscriber and publisher.
ros::Publisher steering_current_pub;
ros::Subscriber accelerating_sub;
ros::Subscriber braking_sub;
ros::Subscriber stellgroessen_should_sub;
//Declaration of functions.
void analogCallback(const std_msgs::Float64::ConstPtr& msg);
std::string convertCharArrayToString(char array[BUFLEN], int size);
std::string convertDoubleToString(double value);
double getValueFromMsg(std::string msg);
void handleReceivedMsg(std::string msg);
void printErrorAndFinish(std::string reason);
void setUpNetwork();
void setUpRosInterface(ros::NodeHandle* node);
void stellgroessenCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg);

int main(int argc, char** argv){
  //Init ros.
  ros::init(argc, argv, "steering_test");
  ros::NodeHandle node;
  //Getting parameters.
  node.getParam("/erod/MAX_STEERING_ANGLE", MAX_STEERING_ANGLE);
  node.getParam("/topic/STELLGROESSEN_SAFE", STELLGROESSEN_TOPIC);
  node.getParam("/topic/STATE_STEERING_ANGLE", STEERING_CURRENT_TOPIC);
  //Initialise addresses.
  setUpNetwork();
  if ((sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) printErrorAndFinish("socket");
  if(bind(sock, (struct sockaddr*)&si_me, sizeof(si_me)) == -1) printErrorAndFinish("binding");
  //Initialise ros interface.
  setUpRosInterface(&node);
  //Listening for and sending data.
  while(ros::ok()){
    //Receiving.
    // int recv_len;
    // char buffer_in[BUFLEN];
    // if ((recv_len = recvfrom(sock, buffer_in, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == -1) 
    //        printErrorAndFinish("receiving");
    // std::string buffer_in_string = convertCharArrayToString(buffer_in, recv_len);
    // std::cout << "Received: " << buffer_in_string << std::endl;
    // handleReceivedMsg(buffer_in_string);
    //Sending.
    ros::spinOnce();
  }
  return 0;
}

void analogCallback(const std_msgs::Float64::ConstPtr& msg){
  //Sending accelerating voltage.
  double voltage = msg->data; 
  if(voltage < 0){
    std::string acc_string = "ab:" + convertDoubleToString(-voltage);
    const char *buffer_out = acc_string.c_str();
    if (sendto(sock, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_NI, slen) == -1) 
        printErrorAndFinish("sending analog input");  
  }
  if(voltage > 0){
  std::string acc_string = "aa:" + convertDoubleToString(voltage);
  // convertStringToCharArray(acc_string, buffer_out);
  const char *buffer_out = acc_string.c_str();
  if (sendto(sock, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_NI, slen) == -1) 
        printErrorAndFinish("sending analog input");  
  }
  
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
  std::string kind(msg, 0, 1);
  double value = getValueFromMsg(msg);
  //Creating ros msg.
  std_msgs::Float64 ros_msg;
  ros_msg.data = value;
  //Answers.
  if(kind == "si") steering_current_pub.publish(ros_msg);
  else std::cout<<"ARC INTERFACE: Cannot assign msg " << msg << std::endl;
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
  si_NI.sin_addr.s_addr = inet_addr("10.0.0.5");
  //Socket length.
  slen = sizeof(si_other);
}

void setUpRosInterface(ros::NodeHandle* node){
  //Publisher and subscriber
  steering_current_pub = node->advertise<std_msgs::Float64>(STEERING_CURRENT_TOPIC, 10);
  accelerating_sub = node->subscribe("/analog_acc", 10, analogCallback);
  stellgroessen_should_sub = node->subscribe(STELLGROESSEN_TOPIC, 10, stellgroessenCallback);
  //Sending initialised comment.
  std::cout << std::endl << "ARC INTERFACE: Ros core initialised "<< std::endl;
}

void stellgroessenCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg){
  //Sending steering angle.
  double steering_should = msg->steering_angle + MAX_STEERING_ANGLE; 
  std::string steering_string = "ss:" + convertDoubleToString(steering_should);
  const char *buffer_out = steering_string.c_str();
  if (sendto(sock, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_NI, slen) == -1) 
        printErrorAndFinish("sending steering_should");  
}