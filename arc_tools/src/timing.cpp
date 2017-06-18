#include "arc_tools/timing.hpp"

using namespace arc_tools;

Clock::Clock() { start(); first_step_ = true;}

double Clock::getTimestep(){
  if(first_step_){
    last_time_ = getTime();
    first_step_ = false;
  }
  current_time_ = getTime();
  time_step_ = current_time_ - last_time_;
  last_time_ = getTime();
  return time_step_ * kMilisecondsToSeconds;
}

double Clock::getTimeFromStart(){
  if(first_step_){
    last_time_ = getTime();
    first_step_ = false;
  }
  current_time_ = getTime();
  time_step_ = current_time_ - last_time_;
  return time_step_ * kMilisecondsToSeconds;
}

void Clock::start(){ gettimeofday(&real_time_start_, NULL); }

double Clock::getTime() { takeTime(); return getRealTime(); }

double Clock::getRealTime() { return real_time_ms_;}

void Clock::takeTime(){
    //Updating cpu Time
  struct timeval end;
  gettimeofday(&end, NULL);
  long seconds, useconds;
  seconds  = end.tv_sec  - real_time_start_.tv_sec;
  useconds = end.tv_usec - real_time_start_.tv_usec;
  real_time_ms_ = (seconds * kSecondsToMiliseconds +
                   useconds * kMicrosecondsToMiliseconds) + 0.5;
}
