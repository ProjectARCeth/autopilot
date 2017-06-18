#ifndef ___ROSINTERFACE_HPP___
#define ___ROSINTERFACE_HPP___

#include "arc_msgs/State.h"
#include "arc_tools/coordinate_transform.hpp"

#include <ackermann_msgs/AckermannDrive.h>
#include "Eigen/Dense"
#include <iostream>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <vector>

#include <QMutex>
#include <QStringList>
#include <QtCore>
#include <QThread>

class RosInterface : public QObject {
	Q_OBJECT
public:
    RosInterface(int argc, char **pArgv);
    virtual ~RosInterface();
    void changeToManuell();
    bool getInitMode();
    bool init();
    void launchCommandCallback(const std_msgs::Bool::ConstPtr& msg);
    void launchingInfoCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);
    void launching();
    void devCallback(const std_msgs::Float64::ConstPtr& msg);
    void devVelCallback(const std_msgs::Float64::ConstPtr& msg);
    void obstacleDistanceCallback(const std_msgs::Float64::ConstPtr& msg);
    void notstop();
    void notstopCallback(const std_msgs::Bool::ConstPtr& msg);
    void purePursuitCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void readyForLaunchingCallback(const std_msgs::Bool::ConstPtr& msg);
    void shutdown();
    void steeringCallback(const std_msgs::Float64::ConstPtr& msg);
    void stellgroessenCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg);
    void stateCallback(const arc_msgs::State::ConstPtr& msg);
    void wheelLeftCallback(const std_msgs::Float64::ConstPtr& msg);
    void wheelRightCallback(const std_msgs::Float64::ConstPtr& msg);
    Q_SLOT void run();
    Q_SIGNAL void newDev(double deviation);
    Q_SIGNAL void newObstacleDis(double distance);
    Q_SIGNAL void newLaunchCommand();
    Q_SIGNAL void newLaunching1(bool gps, bool vi, bool velodyne, bool rovio, 
                                bool state_estimation, bool orbslam);
    Q_SIGNAL void newLaunching2(bool controlling, bool ni_client, bool obstacle_detection, bool guard);
    Q_SIGNAL void newNotstop();
    Q_SIGNAL void newPathInfo(float dis_start, float dis_end, float x_local, float y_local);
    Q_SIGNAL void newSteering(double angle);
    Q_SIGNAL void newState(double x, double y, double velocity, int array_position);
    Q_SIGNAL void newStellgroessen(double vel_should, double steering_should);
    Q_SIGNAL void newVelDev(double vel_deviation);
    Q_SIGNAL void newVelInfo(float v_ref, float bound_phys, float bound_teach, float v_final);
    Q_SIGNAL void newWheelLeft(double wheel_left);
    Q_SIGNAL void newWheelRight(double wheel_right);

private:
    int init_argc_;
    char** init_argv_;
    //Publisher and subscriber.
    ros::Publisher launch_command_pub_;
    ros::Publisher notstop_pub_;
    ros::Publisher shutdown_pub_;
    ros::Publisher vcu_ping_pub_;
    ros::Subscriber deviation_sub_;
    ros::Subscriber deviation_vel_sub_;
    ros::Subscriber launching_info_sub_;
    ros::Subscriber obstacle_distance_sub_;
    ros::Subscriber notstop_sub_;
    ros::Subscriber pure_pursuit_info_sub_;
    ros::Subscriber ready_for_launching_sub_;
    ros::Subscriber steering_sub_;
    ros::Subscriber stellgroessen_sub_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber wheel_left_sub;
    ros::Subscriber wheel_right_sub;
    //Yaml constants.
    bool MODE_INIT;
    int MIN_PUBLISH_NOTSTOP_COUNT;
    //Thread.
    QThread * thread_;
};
#endif

