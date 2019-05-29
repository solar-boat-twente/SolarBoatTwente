


/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Button_Encoder.hpp
 * Author: Yvon
 *
 * Created on May 7, 2019, 10:25 AM
 */

#ifndef ENCODERS_HPP
#define ENCODERS_HPP


#include <vector>
#include <stdint.h>
#include <thread>

//#include "../structures.h"
#include "../../solarboattwente.h"
#include "../../lib-cpp/Serial/Serial.h"


namespace MIO{

namespace UI{

const short int STD_BUTTON_DELAY = 10;
const char BUTTON_STOP_BYTES[2] = {'\r', '\n'};
const short int BUTTON_STOP_LENGTH = 2;
const short int BUTTON_MSG_LENGTH = 3;

//TODO (sander): THIS NEEDS TO BE CHANGED FOR THE REAL VALUE
const short int BUTTON_DELAY = 500;

class ButtonEncoder{
  
 public:
  struct ButtonEncoderStatus{
    
    Serial::SerialStatus * serial_state;
    
    bool read_state;
  };
  
 public:
  ButtonEncoder(Serial * serial);
  
  int start_reading(structures::UserInput * user_input, short int delay = STD_BUTTON_DELAY);
  
  int stop_reading();
  
 private:
  int get_data_(structures::UserInput * const user_input);
  
  int parse_data_(std::vector<uint8_t> *bytes, structures::UserInput * const user_input);
  
  void reading_thread_(structures::UserInput * const user_input, short int delay);
  
  Serial * const serial_;
  
  ButtonEncoderStatus * const button_encoder_status = new ButtonEncoderStatus;
  
  std::thread m_reading_thread_;
  
  structures::UserInput * user_input = NULL; 
  
}; 
}
}





#endif /* ENCODERS_HPP */

