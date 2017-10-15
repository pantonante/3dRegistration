#ifndef CPU_TIMER_H
#define CPU_TIMER_H

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <time.h>

#ifndef CLOCK_REALTIME
  #define CLOCK_REALTIME CLOCK_MONOTONIC
#endif

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#ifdef _WIN32
#include <wintime.h>
#else
#include <sys/time.h>
#endif

typedef std::pair<std::string,float> TimingInfo;
typedef std::map<std::string,float>::iterator it_type;

class CPUTimer {
 private:
  timespec startTime;
  timespec endTime;

  bool startWasSet;
  bool stopWasSet;

  void getTime(struct timespec *ts);
  timespec toc();

  std::map<std::string,float> timingsMap;

 public:
  CPUTimer();
  ~CPUTimer();

  void tic();
  void toc(std::string name);

  float totalTiming();
  std::vector<TimingInfo> getMeasurements();
  std::string allTimings();
};

#endif