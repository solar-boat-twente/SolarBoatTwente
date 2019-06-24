/* 
 * File:   PID_caller.h
 * Author: Arjan
 *
 * Created on 11 maart 2019, 12:23
 */
#ifndef PID_CALLER_H
#define	PID_CALLER_H
#include "pid.h"
#include "../DataStore.h"
#include <fstream>
#include <vector>

//#include "structures.h"
namespace MIO{
namespace control{
class PID_caller {
  
  struct PIDRoll {
    double dt;
    double Kp;
    double Kd;
    double Ki;
    double Fc;
};

 public:

  PID_caller(DataStore * const control_data);
  
  PID_caller(const PID_caller& orig);
  
  virtual ~PID_caller();
  
  void set_PID_from_config(std::vector<int>& pid_roll, std::vector<int>& pid_height);

  void PID_in(structures::FlyMode fly_mode = structures::FLY);

  
 private:
  DataStore* const control_data_;
  PID * pid_roll;
  PID * pid_pitch;
  PID * pid_height;
  std::ofstream file_;
  
};
}
}
#endif	/* PID_CALLER_H */

