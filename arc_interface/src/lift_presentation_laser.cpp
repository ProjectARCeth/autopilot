#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <arpa/inet.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>

#include <iostream>
#include <vector>

#include <sensor_msgs/PointCloud2.h>

//Cloud.
pcl::PointCloud<pcl::PointXYZI> label_cloud;
//Publisher and Subscriber.
ros::Publisher labeled_cloud_pub;
ros::Subscriber cloud_sub;

sensor_msgs::PointCloud2 cloudToMsg(const pcl::PointCloud<pcl::PointXYZI> cloud){
  sensor_msgs::PointCloud2 msg;
  pcl::PCLPointCloud2 temp_pcl22;
    pcl::toPCLPointCloud2(cloud, temp_pcl22);
    pcl_conversions::fromPCL(temp_pcl22, msg);
    msg.header.stamp = ros::Time::now();
    return msg;
}

pcl::PointCloud<pcl::PointXYZI> msgToCloud(const sensor_msgs::PointCloud2& msg){
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PCLPointCloud2 temp_pcl2;
    pcl_conversions::toPCL(msg, temp_pcl2);
    pcl::fromPCLPointCloud2(temp_pcl2, cloud);
    return cloud;
}

void cloudCallback(const sensor_msgs::PointCloud2& msg){
  //Clear vectors.
  label_cloud.clear();
  //Transform msg to xyzi cloud.
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud = msgToCloud(msg);
  //Transforming into x,y,z vectors.
  std::vector<double> x,y,z, intensity;
  for(int i = 0; i<cloud.size(); ++i){
    x.push_back(cloud.points[i].x);
    y.push_back(cloud.points[i].y);
    z.push_back(cloud.points[i].z);
    intensity.push_back(cloud.points[i].intensity);
  }
  //Select candidate points.
  for(int i = 0; i<cloud.points.size(); ++i){
    //Cones located in limited lane, y constraint.
    if(cloud.points[i].y <= 1.5 && cloud.points[i].y >= -2.4){
    //Cones only in front of car, visual detection lonely possible here.
    if(cloud.points[i].x <= 1.5 && cloud.points[i].x >= -2.5){
    //Due to its white color the cones refelcted beams should have high intensity.
      label_cloud.push_back(cloud.points[i]);
    }}
  }
  //Publish labeled cloud.
  sensor_msgs::PointCloud2 label_cloud_msg = cloudToMsg(label_cloud);
  label_cloud_msg.header.frame_id = msg.header.frame_id;
  labeled_cloud_pub.publish(label_cloud_msg);
}

void printError(std::string error){
    std::cout << "ARC Interface: Error with " + error << std::endl;
}

int main(int argc, char** argv){
  //Init ros.
  ros::init(argc, argv, "lift_presentation_laser");
  ros::NodeHandle node;
  //Send steering zero.
  struct sockaddr_in si_me_, si_other_, si_VCU_;
  int sock_;
  socklen_t slen_;
  memset((char *) &si_me_, 0, sizeof(si_me_));
  si_me_.sin_family = AF_INET;
  si_me_.sin_port = htons(8010);
  si_me_.sin_addr.s_addr = htonl(INADDR_ANY);
  si_VCU_.sin_family = AF_INET;
  si_VCU_.sin_port = htons(8001);
  si_VCU_.sin_addr.s_addr = inet_addr("10.0.0.8");
  slen_ = sizeof(si_other_);
  //Set up vcu udp binding (socket + binding).
  if ((sock_=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) printError("socket");
  if(bind(sock_, (struct sockaddr*)&si_me_, sizeof(si_me_)) == -1) printError("binding");
  //Send steering msg.
  std::string sending = "cc:5";
  const char *buffer_out = sending.c_str();
  if (sendto(sock_, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_VCU_, slen_) == -1) 
    printError("sending ");
  ros::Duration(0.5).sleep();
  sending = "am:1";
  const char *buffer_out_am = sending.c_str();
  if (sendto(sock_, buffer_out_am, sizeof(buffer_out_am), 0, (struct sockaddr*) &si_VCU_, slen_) == -1) 
    printError("sending ");
  ros::Duration(0.5).sleep();
  sending = "ss:1003";
  const char *buffer_out_ss = sending.c_str();
  if (sendto(sock_, buffer_out_ss, sizeof(buffer_out_ss), 0, (struct sockaddr*) &si_VCU_, slen_) == -1) 
    printError("sending ");
  ros::Duration(2.5).sleep();
  sending = "am:0";
  const char *buffer_out_am_out = sending.c_str();
  if (sendto(sock_, buffer_out_am_out, sizeof(buffer_out_am_out), 0, (struct sockaddr*) &si_VCU_, slen_) == -1) 
    printError("sending ");
  //Init pubs & subs.
  labeled_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/labeled_points", 10);
  cloud_sub = node.subscribe("/velodyne_points", 10, cloudCallback);
  //Spinning.
  ros::spin();
  return 0;

}