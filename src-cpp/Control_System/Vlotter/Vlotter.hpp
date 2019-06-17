/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Vlotter.hpp
 * Author: Sander Oosterveld
 *
 * Created on June 13, 2019, 10:21 PM
 */

#ifndef VLOTTER_HPP
#define VLOTTER_HPP

#include "../../../lib-cpp/Serial/Serial.h"
#include <thread>

namespace MIO{
namespace Control{

const char VLOTTER_STOP_BYTES[2] = {'\r', '\n'};
const int VLOTTER_STOP_LENGTH = 2;
const int VLOTTER_MSG_LENGTH = 4;
const int VLOTTER_BEAM_LENGTH = 1;

enum EncoderNumber{
  ENCODER_LEFT,
  ENCODER_RIGHT
};

class Vlotter{
  
 public:
  Vlotter(Serial * serial) : serial_(serial) {

  };
  
  void start_reading(short int delay = 50);
  
  void stop_reading();
  
  float get_angle_deg(EncoderNumber encoder);
  
  float get_height_deg(EncoderNumber encoder);
  
 private:
  
  void reading_thread_(short int delay);
  
  void read_vlotter_data_(uint8_t buffer[]);
  
  int bytes_to_int_(uint8_t bytes[]);
  
  float calculate_angle_(int value);
  
  float calculate_height_(float angle);
  
  Serial * const serial_;
  
  std::thread thread_;
  bool thread_active_;
  
  float angle_left_;
  float angle_right_;
  
  float height_left_;
  float height_right_;
  
  
  
  
  
    
  
};


}
}



#endif /* VLOTTER_HPP */

