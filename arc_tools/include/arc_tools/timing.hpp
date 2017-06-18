#ifndef TIMING_ARC_TOOLS_HPP
#define TIMING_ARC_TOOLS_HPP

#include <iostream>
#include <sys/time.h>

namespace arc_tools{

class Clock {
 public:

  Clock();
  double getTimestep();
  double getTimeFromStart();
  void start();
 private:
  double getTime();
  double getRealTime();
  void takeTime();
  struct timeval real_time_start_;
  double real_time_ms_;
  bool first_step_;
  double last_time_, current_time_, time_step_;
  static const double kSecondsToMiliseconds = 1000.0;
  static const double kMicrosecondsToMiliseconds = 0.001;
  static const double kMilisecondsToSeconds = 0.001;
};
}//namespace arc_tools.

#endif
