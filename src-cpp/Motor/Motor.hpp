/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Motor.hpp
 * Author: Sander Oosterveld
 *
 * Created on May 28, 2019, 10:21 PM
 */

#ifndef MOTOR_HPP
#define MOTOR_HPP


#include "../../lib-cpp/Canbus/canbus.h"
#include "../../solarboattwente.h"

#include "Motor_defs.hpp"

namespace MIO{
namespace PowerElectronics{

const int kMotorStandardURange = 100;
const int kMotorStandardIRange = 100;

class Motor{
 public:
  /**
   * Motor class which wraps the data from the motor nicely. Using the public structure
   * values you can get the data directly. 
   * Moreover one can write the data from the motor to the power_input structure directly
   * @param driver_can A pointer to a canbus object 
   * @param read_flag Flag of what you want to read. posibilities are: 
   *  RD_SUPPLY1, RD_MOTOR1, RD_MOTOR2, RD_DRIVERSTATE, RD_RANGEREF
   */
  Motor(CANbus * const driver_can, int read_flag = 0);
  
  /**
   * Reads out the data from the motor and updates the internal values structure
   * @return success
   */
  bool update_data_from_driver();

  /**
   * Send message to the motor to setup the sampling
   */
  void setup_sampling();

  /**
   * Update the internal data from structures: not yet used might be used to send data to 
   * to the motor but that is still odne in the main loop
   * @return 
   */
  bool update_data_from_structs();

  /**
   * Update power structures from the interval values
   * @param power_input pointer to the power_input structure to be updated.
   */
  void update_power_input(structures::PowerInput * power_input);

  /**
   * Not yet used but can later be used to write the speed message to the driver
   */
  void write_to_driver();
    //Different values that can be gotten from the motor: initialized at -1. So when
  // the motor is off that is no problem
  struct MotorValues {
    float supply_current = -1;
    float supply_voltage = -1;
    two_bytes low_priority_limiter = {-1};
    two_bytes error_word = {-1};
    uint8_t high_priority_limiter = -1;

    float motor_power = -1;
    float torque = -1;

    bool driver_state = false;

    int rotor_speed = -1;

    float phase_current = -1;
    float link_voltage = -1;

    two_bytes Urange = {-1};
    two_bytes Irange = {-1};
    two_bytes Uref = {-1};
    two_bytes Iref = {-1};

    uint8_t driver_temp = -1;

    uint8_t motor_mode = -1;
  };
  
  //Public structure values which can be entered directly in order to get the motor data
  MotorValues values;
  
  
 private:
  
  void parse_supply_1(uint8_t data[]);
  
  void parse_motor_1(uint8_t data[]);
  
  void parse_motor_2(uint8_t data[]);
  
  void parse_reference(uint8_t data[]);
  
  void parse_driver_data(uint8_t data[]);

  CANbus * const driver_can_;
  
  int read_flag_; 
  
  bool watchdog;
  
};

}
}
#endif /* MOTOR_HPP */

