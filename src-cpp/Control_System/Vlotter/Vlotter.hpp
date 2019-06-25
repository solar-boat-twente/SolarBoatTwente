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
namespace control{

constexpr char kVlotterStopBytes[2] = {'\r', '\n'};
constexpr int kVlotterStopLength = 2;
constexpr int kVlotterMsgLength = 4;

constexpr float kVlotterBeamLength = 0.7;
constexpr float kVlotterDistance = 1.3;


constexpr auto kVlotterSerialPort = "/dev/vlotter";

enum class EncoderNumber{
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
  Vlotter(Serial * const serial);
  
  Vlotter();
  
  ~Vlotter();
  
  void start_reading(short int delay = 0);
  
  void stop_reading();
  
  float get_angle_rad(EncoderNumber encoder);
  
  float get_height(EncoderNumber encoder);
  
  float get_roll_rad();
   
  void configure(float new_phi_left, float new_phi_right);
  
 private:
  
  void reading_thread_(short int delay);
  
  void read_vlotter_data_(uint8_t buffer[]);
  
  int bytes_to_int_(uint8_t bytes[]);
  
  float compute_angle_(int value);
  
  float compute_height_(float angle);
  
  float compute_roll_(float height_left, float height_right);
  
  Serial * const serial_;
  
  std::thread thread_;
  bool thread_active_;
  
  float angle_left_;
  float angle_right_;
  
  float height_left_;
  float height_right_; 
  
  float roll_;
  
  float phi_zero_left_;
  float phi_zero_right_;
  
  float kPhiZeroAngleLeft = 2;
  float kPhiZeroAngleRight = 3;
};


}
}




#endif /* VLOTTER_HPP */

