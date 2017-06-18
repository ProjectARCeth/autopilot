#include "RosInterface.hpp"

std::string DEVIATION_TOPIC;
std::string DEVIATION_VELOCITY_TOPIC;
std::string GUI_NOTSTOP_TOPIC;
std::string LAUNCHING_INFO_TOPIC;
std::string LAUNCHING_COMMAND_TOPIC;
std::string NOTSTOP_TOPIC;
std::string OBSTACLE_DISTANCE_TOPIC;
std::string PURE_PURSUIT_INFO_TOPIC;
std::string READY_FOR_LAUNCHING_TOPIC;
std::string SHUTDOWN_TOPIC;
std::string STATE_TOPIC;
std::string STEERING_TOPIC;
std::string STELLGROESSEN_TOPIC;
std::string VCU_PING_TOPIC;
std::string WHEEL_LEFT_TOPIC;
std::string WHEEL_RIGHT_TOPIC;
int QUEUE_LENGTH;

RosInterface::RosInterface(int argc, char **pArgv){
    init_argc_ = argc;
    init_argv_ = pArgv;
    MODE_INIT = true;
    if(strlen(*(init_argv_ + 1)) == 5) MODE_INIT = false;
}

RosInterface::~RosInterface(){
    if (ros::isStarted()){
        ros::shutdown();
        ros::waitForShutdown();
    }
    thread_->wait();
}

void RosInterface::changeToManuell(){
    std::cout << std::endl << "GUI: Manuell Mode " << std::endl;
    std_msgs::Bool anti_launching_msg;
    anti_launching_msg.data = false;
    launch_command_pub_.publish(anti_launching_msg);
}

bool RosInterface::getInitMode(){return MODE_INIT;}

bool RosInterface::init(){
    //Init thread.
    thread_ = new QThread();
    this->moveToThread(thread_);
    connect(thread_, &QThread::started, this, &RosInterface::run);
    //Init ros.
    ros::init(init_argc_,init_argv_, "arc_gui");
    if (!ros::master::check()) return false;
    ros::start();
    ros::Time::init();
    ros::NodeHandle node;
    //Get parameters.
    node.getParam("/safety/MIN_PUBLISH_NOTSTOP_COUNT", MIN_PUBLISH_NOTSTOP_COUNT);
    node.getParam("/general/QUEUE_LENGTH", QUEUE_LENGTH);
    node.getParam("/topic/NAVIGATION_INFO", PURE_PURSUIT_INFO_TOPIC);
    node.getParam("/topic/NOTSTOP", NOTSTOP_TOPIC);
    node.getParam("/topic/GUI_STOP", GUI_NOTSTOP_TOPIC);
    node.getParam("/topic/OBSTACLE_DISTANCE", OBSTACLE_DISTANCE_TOPIC);
    node.getParam("/topic/READY_FOR_DRIVING", READY_FOR_LAUNCHING_TOPIC);
    node.getParam("/topic/RUNNING_PROGRAMMES", LAUNCHING_INFO_TOPIC);
    node.getParam("/topic/SHUTDOWN", SHUTDOWN_TOPIC);
    node.getParam("/topic/STATE", STATE_TOPIC);
    node.getParam("/topic/STATE_STEERING_ANGLE", STEERING_TOPIC);
    node.getParam("/topic/STELLGROESSEN_SAFE", STELLGROESSEN_TOPIC);
    node.getParam("/topic/TRACKING_ERROR", DEVIATION_TOPIC);
    node.getParam("/topic/TRACKING_ERROR_VELOCITY", DEVIATION_VELOCITY_TOPIC);
    node.getParam("/topic/VCU_LAUNCHING_COMMAND", LAUNCHING_COMMAND_TOPIC);
    node.getParam("/topic/VCU_WORKING_INTERFACE", VCU_PING_TOPIC);
    node.getParam("/topic/WHEEL_REAR_LEFT", WHEEL_LEFT_TOPIC);
    node.getParam("/topic/WHEEL_REAR_RIGHT", WHEEL_RIGHT_TOPIC);
    //Init publisher and subscriber.
    launch_command_pub_ = node.advertise<std_msgs::Bool>(LAUNCHING_COMMAND_TOPIC, QUEUE_LENGTH);
    notstop_pub_ = node.advertise<std_msgs::Bool>(GUI_NOTSTOP_TOPIC, QUEUE_LENGTH);
    shutdown_pub_ = node.advertise<std_msgs::Bool>(SHUTDOWN_TOPIC, QUEUE_LENGTH);
    vcu_ping_pub_ = node.advertise<std_msgs::Float64>(VCU_PING_TOPIC, QUEUE_LENGTH);
    deviation_sub_ = node.subscribe(DEVIATION_TOPIC, QUEUE_LENGTH, &RosInterface::devCallback, this);
    deviation_vel_sub_ = node.subscribe(DEVIATION_VELOCITY_TOPIC, QUEUE_LENGTH, &RosInterface::devVelCallback, this);
    obstacle_distance_sub_ = node.subscribe(OBSTACLE_DISTANCE_TOPIC, QUEUE_LENGTH, &RosInterface::obstacleDistanceCallback, this);
    launching_info_sub_ = node.subscribe(LAUNCHING_INFO_TOPIC, QUEUE_LENGTH, &RosInterface::launchingInfoCallback, this);
    notstop_sub_ = node.subscribe(NOTSTOP_TOPIC, QUEUE_LENGTH, &RosInterface::notstopCallback, this);
    pure_pursuit_info_sub_ = node.subscribe(PURE_PURSUIT_INFO_TOPIC, QUEUE_LENGTH, &RosInterface::purePursuitCallback, this);
    ready_for_launching_sub_ = node.subscribe(READY_FOR_LAUNCHING_TOPIC, QUEUE_LENGTH, &RosInterface::readyForLaunchingCallback, this);
    steering_sub_ = node.subscribe(STEERING_TOPIC, QUEUE_LENGTH, &RosInterface::steeringCallback, this);
    stellgroessen_sub_ = node.subscribe(STELLGROESSEN_TOPIC, QUEUE_LENGTH, &RosInterface::stellgroessenCallback, this);
    velocity_sub_ = node.subscribe(STATE_TOPIC, QUEUE_LENGTH, &RosInterface::stateCallback, this);
    wheel_left_sub = node.subscribe(WHEEL_LEFT_TOPIC, QUEUE_LENGTH, &RosInterface::wheelLeftCallback, this);
    wheel_right_sub = node.subscribe(WHEEL_RIGHT_TOPIC, QUEUE_LENGTH, &RosInterface::wheelRightCallback, this);
    //Start thread.
    thread_->start();
    return true;
}

void RosInterface::run(){
    ros::Rate loop_rate(100);
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void RosInterface::launching(){
    std::cout << std::endl << "GUI: System launched " << std::endl;
    if(MODE_INIT){
        std_msgs::Float64 ping_msg;
        ping_msg.data = 5;
        vcu_ping_pub_.publish(ping_msg);
        ros::Duration(0.5).sleep();
        std_msgs::Bool launching_msg;
        launching_msg.data = true;
        launch_command_pub_.publish(launching_msg);
    }
}

void RosInterface::notstop(){
    std::cout << std::endl << "GUI: NOTSTOP !!!!!!!!" << std::endl;
    std_msgs::Bool notstop_msg;
    notstop_msg.data = true;
    std::cout << "Start: " << ros::Time::now() << std::endl;
    for (int i=0; i<MIN_PUBLISH_NOTSTOP_COUNT; ++i) notstop_pub_.publish(notstop_msg);
}

void RosInterface::shutdown(){
    std::cout << std::endl << "GUI: Controlled Shutdown !!!" << std::endl;
    std_msgs::Bool shutdown_msg;
    shutdown_msg.data = true;
    for (int i=0; i<MIN_PUBLISH_NOTSTOP_COUNT; ++i) shutdown_pub_.publish(shutdown_msg);
}

void RosInterface::devCallback(const std_msgs::Float64::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    double deviation = msg->data;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newDev(deviation);
}

void RosInterface::devVelCallback(const std_msgs::Float64::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    double vel_deviation = msg->data;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newVelDev(vel_deviation);
}

void RosInterface::launchingInfoCallback(const std_msgs::Int16MultiArray::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    bool controlling = msg->data[0];
    bool gps = msg->data[1];
    bool ni_client = msg->data[2];
    bool obstacle_detection = msg->data[3];
    bool guard = msg->data[4];
    bool rovio = msg->data[5];
    bool state_estimation = msg->data[6];
    bool orbslam = msg->data[7];
    bool velodyne = msg->data[8];
    bool vi = msg->data[9];
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newLaunching1(gps, vi, velodyne, rovio, state_estimation, orbslam);
    Q_EMIT newLaunching2(controlling, ni_client, obstacle_detection, guard);
}

void RosInterface::obstacleDistanceCallback(const std_msgs::Float64::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    double distance = msg->data;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newObstacleDis(distance);
}

void RosInterface::notstopCallback(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data == true) Q_EMIT newNotstop();
}

void RosInterface::purePursuitCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    float info[10];
    for (int i = 0; i <= 7; ++i) info[i] = msg->data[i];
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newPathInfo(info[0], info[1], info[2], info[3]);
    Q_EMIT newVelInfo(10.0, info[5], info[6], info[7]);
}

void RosInterface::readyForLaunchingCallback(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data == true) Q_EMIT newLaunchCommand();
}

void RosInterface::steeringCallback(const std_msgs::Float64::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    double angle = msg->data*180/M_PI;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newSteering(angle);
}

void RosInterface::stateCallback(const arc_msgs::State::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double vel = msg->pose_diff;
    int array_position = msg->current_arrayposition;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newState(x, y, vel, array_position);
}

void RosInterface::stellgroessenCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    double steering_should = msg->steering_angle*180/M_PI;
    double vel_should = msg->speed;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newStellgroessen(vel_should, steering_should);
}

void RosInterface::wheelLeftCallback(const std_msgs::Float64::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    double wheel_left = msg->data;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newWheelLeft(wheel_left);
}

void RosInterface::wheelRightCallback(const std_msgs::Float64::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    double wheel_right = msg->data;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newWheelRight(wheel_right);
}