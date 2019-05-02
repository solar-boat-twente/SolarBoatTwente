/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Control Wheel.hpp
 * Author: Sander Oosterveld
 *
 * Created on April 19, 2019, 10:46 AM
 */

#ifndef CONTROL_WHEEL_HPP
#define CONTROL_WHEEL_HPP

#include <vector>
#include <stdint.h>
#include "../Wrappers/Serial.h"
#include <thread>

#include "../structures.h"

namespace MIO{

namespace UI{

const short int STD_CONTROL_WHEEL_DELAY = 10;
const char CONTROL_WHEEL_STOP_BYTES[2] = {'\r', '\n'};
const short int CONTROL_WHEEL_STOP_LENGTH = 2;
const short int CONTROL_WHEEL_MSG_LENGTH = 4;

//TODO (sander): THIS NEEDS TO BE CHANGED FOR THE REAL VALUE
const short int CONTROL_WHEEL_DELAY = 500;

class ControlWheel{
  
 public:
  struct ControlWheelStatus{
    
    Serial::SerialStatus * serial_state;
    
    bool read_state;
  };
  
 public:
  ControlWheel(Serial * serial);
  
  int start_reading(structures::UserInput * user_input, short int delay = STD_CONTROL_WHEEL_DELAY);
  
  int stop_reading();
  
 private:
  int get_data_(structures::UserInput * const user_input);
  
  int parse_data_(std::vector<uint8_t> *bytes, structures::UserInput * const user_input);
  
  void reading_thread_(structures::UserInput * const user_input, short int delay);
  
  Serial * const serial_;
  
  ControlWheelStatus * const control_wheel_status = new ControlWheelStatus;
  
  std::thread m_reading_thread_;
  
  structures::UserInput * user_input = NULL; 
  
}; 
}
}




#endif /* CONTROL_WHEEL_HPP */

