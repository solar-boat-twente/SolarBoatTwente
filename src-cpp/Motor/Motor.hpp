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

const int STD_URANGE = 100;
const int STD_IRANGE = 100;

class Motor{
 public:
  
  Motor(CANbus * driver_can, int read_flag = 0);
  

  bool update_data_from_driver();

  void setup_sampling();

  bool update_data_from_structs();

  void update_power_input(structures::PowerInput * power_input);

  void write_to_driver();
    
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
  
  MotorValues values;
  
  
 private:
  
  void parse_supply_1(uint8_t data[]);
  
  void parse_motor_1(uint8_t data[]);
  
  void parse_motor_2(uint8_t data[]);
  
  void parse_reference(uint8_t data[]);
  
  void parse_driver_data(uint8_t data[]);

  CANbus * driver_can_;
  
  int read_flag_; 
  
  bool watchdog;
  
};

}
}
#endif /* MOTOR_HPP */

