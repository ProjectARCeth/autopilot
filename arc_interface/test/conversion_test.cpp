#include <arpa/inet.h>
#include <iostream>
#include <opencv/cv.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

const int BUFLEN = 512;
//Declaration of functions.
std::string convertCharArrayToString(char array[BUFLEN], int size);
std::string convertDoubleToString(double value);
void convertStringToCharArray(std::string line, char* buffer);
double getValueFromMsg(std::string msg);

int main(int argc, char** argv){
  //Init ros.
  ros::init(argc, argv, "arc_interface_test");
  ros::NodeHandle node;
  //Testing.
  char array[BUFLEN];
  std::string str = "vi:5.37123";
  convertStringToCharArray(str, array);
  std::cout << array << std::endl;
  double value = getValueFromMsg(str);
  std::cout << convertDoubleToString(value) << std::endl;
  //Getting temperature.
  for(int i=0; i<100; ++i) system("sensors");
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

void convertStringToCharArray(std::string line, char* buffer){
  for (int i=0; i<line.size(); ++i){
    buffer[i] = line[i];
  }
}

double getValueFromMsg(std::string msg){
  //Get value string (substracting ; and beginning behind : ).
  std::string value_string(msg, 3, msg.length()-2);
  //Convert to number.
  char buffer[BUFLEN];
  convertStringToCharArray(value_string, buffer);
  double value = atof(buffer);
  return value;
}
