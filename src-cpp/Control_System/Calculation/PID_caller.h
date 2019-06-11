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

//#include "structures.h"
namespace MIO{
namespace Control{
class PID_caller {
  
  struct PIDRoll {
    double dt;
    double Kp;
    double Kd;
    double Ki;
    double Fc;
};

 public:

  PID_caller();
  PID_caller(const PID_caller& orig);
  virtual ~PID_caller();

  void PID_in(structures::FlyMode fly_mode);
  
 private:
  DataStore* m_complementary_data;
  DataStore* m_PID_data;
  PID * pid_roll;
  PID * pid_pitch;
  PID * pid_height;
};
}
}
#endif	/* PID_CALLER_H */

