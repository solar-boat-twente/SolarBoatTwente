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
  /**
   * Class defining both the vlotters, one shall start reading out the function using the
   * start_reading function.
   * After the thread as started one can get the angle (NOT YET THE HEIGHT) in  radians
   * using the get_angle_rad. This one is updated every 50 ms (as defined in the nucleo)
   * @example 
   * #include "src-cpp/Control_System/Vlotter/Vlotter.hpp"\n
   * Serial * serial_vlotter = new Serial("/dev/ACM0");\n
   * Vlotter vlotter(serial_vlotter);\n
   * vlotter.start_reading();\n
   * float angle\n
   * while (true) {\n
   * angle = get_angle_rad();\n
   * (Do some other stuff)\n
   * (wait for some time)\n
   * }
   * vlotter.stop_reading();\n
   * @param serial
   */
  Vlotter(Serial * serial) : serial_(serial) {

  };
  
  void start_reading(short int delay = 0);
  
  void stop_reading();
  
  float get_angle_rad(EncoderNumber encoder);
  
  float get_height(EncoderNumber encoder);
  
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

