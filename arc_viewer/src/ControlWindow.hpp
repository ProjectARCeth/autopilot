#ifndef CONTROL_WINDOW_H
#define CONTROL_WINDOW_H

#include <iostream>

#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QPalette>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextStream>
#include <QtWidgets>
#include <vector>

#include "RosInterface.hpp"

class ControlWindow : public QWidget
{
    Q_OBJECT

public:
    ControlWindow(int argc, char** argv, QWidget * parent = 0);
    Q_SLOT void converting();
    Q_SLOT void notstop();
    Q_SLOT void popUpNotstop();
    Q_SLOT void setLaunchable();
    Q_SLOT void shutdown();
    Q_SLOT void updateDevDisplay(double deviation);
    Q_SLOT void updateLaunchedProgrammes1(bool gps, bool vi, bool velodyne, bool rovio,
                                          bool state_estimation, bool orbslam);
    Q_SLOT void updateLaunchedProgrammes2(bool controlling, bool ni_client, bool obstacle_detection,
                                          bool guard);
    Q_SLOT void updateObstacleDisDisplay(double distance);
    Q_SLOT void updatePathInfoDisplay(float dis_begin, float dis_end, float radius, float ref_steering);
    Q_SLOT void updateSteeringDisplay(double angle);
    Q_SLOT void updateStellgroessenDisplay(double vel_should, double steering_should);
    Q_SLOT void updateStateDisplay(double x, double y, double velocity, int array_position);
    Q_SLOT void updateVelDevDisplay(double deviation);
    Q_SLOT void updateVelInfoDisplay(float ref_vel, float bound_phys, float braking_dis, float bound_teach);
    Q_SLOT void updateWheelLeftDisplay(double wheel_left);
    Q_SLOT void updateWheelRightDisplay(double wheel_right);

private:
    //Init mode.
    bool INIT_MODE;
    //Main layouts.
    QGridLayout *mainLayout;
    QVBoxLayout *topLeftLayout;
    QVBoxLayout *topRightLayout;
    QHBoxLayout *lowerLeftLayout;
    QHBoxLayout *lowerRightLayout;
    QVBoxLayout *middleRightLayout;
    QVBoxLayout *middleCenterLayout;
    QVBoxLayout *middleLeftLayout;
    //State incl. Velocity display.
    QLineEdit *abs_vel_display_;
    QLineEdit *x_pose_display_;
    QLineEdit *y_pose_display_;
    QLineEdit *array_pose_display_;
    //Path deviation display.
    QLineEdit *dev_display_;
    QLineEdit *vel_dev_display_;
    //Obstacle Distance.
    QLineEdit *obstacle_dis_display_;
    //Pure pursuit infos.
    QLineEdit *distance_start_display_;
    QLineEdit *distance_end_display_;
    QLineEdit *ylocal_display_;
    QLineEdit *xlocal_display_;
    QLineEdit *radius_path_display_;
    QLineEdit *velocity_bound_physical_display_;
    QLineEdit *velocity_bound_teach_display_;
    QLineEdit *v_final_display_;
    //Steering information.
    QLineEdit *steering_ist_display_;
    //Wheel sensor information.
    QLineEdit *wheel_left_display_;
    QLineEdit *wheel_right_display_;
    //Stellgroessen information.
    QLineEdit *velocity_should_display_;
    QLineEdit *steering_should_display_;
    //Stop button.
    QPushButton *stop_button_;
    //Shutdown button.
    QPushButton *shutdown_button_;
    //Mode button and display.
    QLineEdit *mode_display_;
    //Launch Button.
    QPushButton *launch_button_;
    bool system_launched_;
    bool autonomous_mode_;
    //Launched programs.
    bool USE_CONTROLLING;
    bool USE_GPS;
    bool USE_NI_CLIENT;
    bool USE_GUARD;
    bool USE_OBSTACLE_DETECTION;
    bool USE_STATE_ESTIMATION;
    bool USE_VELODYNE;
    bool USE_VI;
    //Launching programms checkbox.
    QLabel *controlling_box_;
    QLabel *gps_box_;
    QLabel *guard_box_;
    QLabel *obstacle_detection_box_;
    QLabel *orbslam_box_;
    QLabel *ni_client_box_;
    QLabel *rovio_box_;
    QLabel *state_estimation_box_;
    QLabel *velodyne_box_;
    QLabel *vi_box_;
    //RosInterface.
    RosInterface RosInterface_;
    //Set layout functions.
    void buildInterface(bool mode);
    void checkAndChangeBox(QLabel *label);
    void deleteWidgets();
    void setLaunchButton(QHBoxLayout *bigLayout);
    void setLaunchingProgrammsDisplay();
    void setModeDisplay(bool mode, QVBoxLayout *bigLayout);
    void setShutdownButton(QVBoxLayout *bigLayout);
    void setStopButton(QVBoxLayout *bigLayout);
    void setToManuellButton();
    void setUpDisplay(QLineEdit *display, std::string info, QVBoxLayout *bigLayout);
    void setUpDisplay(QLineEdit *display, std::string info, QVBoxLayout *bigLayout, 
                      QColor &base_color, QColor &text_color);
    void setUpLaunchableProgrammBox(QLabel *label, std::string name);
    void setVelocityDisplay(QVBoxLayout *bigLayout);
};
#endif

