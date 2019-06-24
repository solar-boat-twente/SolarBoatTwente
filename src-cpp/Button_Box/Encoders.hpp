


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

constexpr short int kStandardEncoderDelay = 10;
constexpr char kStandardEncoderStopBytes[2] = {'\r', '\n'};
constexpr short int kStandardEncoderStopLength = 2;
constexpr short int kButtonMsgLength = 3;


class ButtonEncoder{
  
 public:
  struct ButtonEncoderStatus{
    
    Serial::SerialStatus * serial_state;
    
    bool read_state;
  };
  
 public:
  ButtonEncoder(Serial * serial, structures::UserInput * const user_input);
  
  ~ButtonEncoder();
  
  int start_reading(structures::UserInput * user_input, short int delay = kStandardEncoderDelay);
  
  int stop_reading();
  
 private:
  int get_data_(structures::UserInput * const user_input);
  
  int parse_data_(std::vector<uint8_t> &bytes, structures::UserInput * const user_input);
  
  void reading_thread_(structures::UserInput * const user_input, short int delay);
  
  // Pointer to a serial object used for the encoders
  Serial * const serial_;
  
  
  ButtonEncoderStatus button_encoder_status;
  
  std::thread m_reading_thread_;
  
  
}; 
}
}





#endif /* ENCODERS_HPP */

