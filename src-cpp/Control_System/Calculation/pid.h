/* 
 * File:   pid.h
 * Author: User
 *
 * Created on 27 februari 2019, 10:28
 */

#ifndef _PID_H_
#define _PID_H_
#include <vector>


#include "../DataStore.h"
#include "../../../solarboattwente.h"

namespace MIO{
namespace control{

struct PIDValues {
  float Pout;
  float Iout;
  float Dout;
  float intergral;
};
  
class PIDImpl {
 public:
  PIDImpl( float dt, float Kp, float Kd, float Ki );
  
  ~PIDImpl();
  
  void update_values(float dt, float Kp, float Kd, float Ki);
  
  float calculate( float setpoint, float pv );
  
  PIDValues get_PID_values();
  
  void reset_integral();
  
  void reset_integral_roll();
  
 private: 
  float dt_;
  float max_;
  float min_;
  float Kp_;
  float Kd_;
  float Ki_;
  float pre_error_;
  float integral_;
  
  PIDValues PID_values_;
};

class PID {
  

  
 public:
  /**
   * Class which is used to calculate the required forces using PID controllers
   * You required a seperate PID object for each of the three directions: height, pitch and roll.
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
  PID();
  // Kp -  proportional gain
  // Ki -  Integral gain
  // Kd -  derivative gain
  // dt -  loop interval time
  // max - maximum value of manipulated variable
  // min - minimum value of manipulated variable
  void set_PID_roll(structures::PIDState state);
  void set_PID_pitch(structures::PIDState state);
  void set_PID_height(structures::PIDState state);

  void set_PID_from_config(std::vector<int> config_pid);
  // Returns the manipulated variable given a setpoint and current process value
  float calculate( float setpoint, float pv );
  ~PID();
  
  PIDValues get_PID_values();
  
  void reset_integral();
  
  void reset_integral_roll();
  
 private:
  PIDImpl * const pimpl;
  
  PIDValues PID_values_;
  
  float config_dt = 0.0125;
  int config_P = 0;
  int config_I = 0;
  int config_D = 0;
  

};
}
}
#endif
