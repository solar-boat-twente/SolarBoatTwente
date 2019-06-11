/* 
 * File:   pid.h
 * Author: User
 *
 * Created on 27 februari 2019, 10:28
 */

#ifndef _PID_H_
#define _PID_H_
#include "DataStore.h"
#include "../../../solarboattwente.h"

namespace MIO{
namespace Control{
class PIDImpl {
 public:
  PIDImpl( float dt, float Kp, float Kd, float Ki, float Fc );
  ~PIDImpl();
  
  void update_values(float dt, float Kp, float Kd, float Ki, float Fc);
  
  float calculate( float setpoint, float pv );
  
 private: 
  float dt_;
  float max_;
  float min_;
  float Kp_;
  float Kd_;
  float Ki_;
  float Fc_;
  float pre_error_;
  float integral_;
};

class PID {
 public:
  /**
   * Class which is used to calculate the required forces using PID controllers
   * You required a seperate PID object for each of the three directions: height, pitch and roll.
   * TODO: make a PID object which inherets this and is for the seperate directions..
   * Example use:
   * PID * roll_pid = new PID();
   * roll_pid.set_PID_roll(STATE2);
   * float force_roll;
   * while true {
   *  \\get_values from sensors
   *  force_roll = roll_pid.calculate(setpoint, value from sensors);
   *  \\Send force_roll to actuators
   *  \\Wait a certain dt.
   */
  PID() {
    pimpl = new PIDImpl(0,0,0,0,0);
  };
  // Kp -  proportional gain
  // Ki -  Integral gain
  // Kd -  derivative gain
  // dt -  loop interval time
  // max - maximum value of manipulated variable
  // min - minimum value of manipulated variable
  void set_PID_roll(structures::PIDState state);
  void set_PID_pitch(structures::PIDState state);
  void set_PID_height(structures::PIDState state);

  // Returns the manipulated variable given a setpoint and current process value
  float calculate( float setpoint, float pv );
  ~PID();
  
 private:
  PIDImpl *pimpl;
};
}
}
#endif
