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
  /**
   * Start the reading thread in the thread named thread_
   * @param delay Delay between reading from the serial two times
   */
  void start_reading(short int delay = 0);
  
  /**
   * Waits till the reading thread can stop and then stops it afterwards
   */
  void stop_reading();
  
  /**
   * Return the most recent angle (in radians) gotten from the encoder
   * @param encoder EncoderNumber which you would like to get the angle for call like: 
   *  EncoderNumber::ENCODER_LEFT or EncoderNumber::ENCODER_RIGHT
   * @return angle of the asked for vlotter in radians
   */
  float get_angle_rad(EncoderNumber encoder);
  
  /**
   * Returns the height of the boat measured from the turning point of the vlotters
   * @param Height of the boat according to the vlotters measured from the rotation point of the vlotters
   * down
   * @return Height of the boat in meters 
   */
  float get_height(EncoderNumber encoder);
  
  /**
   * Uses the different heights for both the vlotters to calculate the roll of the boat according to the vlotters
   * @return roll of the boat in radians
   */
  float get_roll_rad();
   
  /**
   * Setup the phi zero angles of the two vlotters. The zero phi angle is the angle when the vlotter is perpendicular to the boat!
   * 
   * @param new_phi_left Phi angle when the boat left vlotter is perpendicular to the boat
   * @param new_phi_right Phi angle when the boat right vlotter is perpendicular to the boat
   */
  void configure(float new_phi_left, float new_phi_right);
  
 private:
  
  /**
   * Function which the thread runs, just a while loop of reading out serial and handeling the input data
   * Stores the height and angle inside the member variables
   * @param delay 
   */
  void reading_thread_(short int delay);
  
  /**
   * Read data from serial and put it in the buffer
   * @param buffer Buffer of characters large enough to fit at least 4 values
   */
  void read_vlotter_data_(uint8_t buffer[]);
  
  /**
   * Go for 12 bit data (2 bytes) to an integer
   * @param bytes array of 2 bytes long, first value is the msb, second is lsb
   * @return integers value gotten from the bytes
   */
  int bytes_to_int_(uint8_t bytes[]);
  
  /**
   * Compute the angle in radians from the raw integer value
   * @param value raw value from 0-4095
   * @return angle in radians
   */
  float compute_angle_(int value);
  
  /**
   * Compute the height from the angle. Uses the constant VlotterBeamLength
   * 
   * @param angle Angle is radians
   * @return  Height in meters
   */
  float compute_height_(float angle);
  
  /**
   * Compute the roll using the two different heights of the vlotter. Uses the arcsin
   * and the distance between the two vlotters
   * 
   * @param height_left Height in meters of left vlotter
   * @param height_right Height in meters of the right vlotter
   * @return Roll in angles where counterclockwise is positive
   */
  float compute_roll_(float height_left, float height_right);
  
  /*Pointer to a Serial object*/
  Serial * const serial_;
  
  /*Thread which is a member variable to allow it to be stopped in another object*/
  std::thread thread_;
  /*Boolean which is required to be able to turn the while loop of the thread on or off*/
  bool thread_active_;
  
  /*Stored angles*/
  float angle_left_;
  float angle_right_;
  
  /*Stored heights*/
  float height_left_;
  float height_right_; 
  
  /*Stored Rolls*/
  float roll_;
  
  /*constant angles which can only be modified in the configuration method*/
  //TODO: make these constant and define in the constructor --> Use a vlotter builder class
  float kPhiZeroAngleLeft = 2.41441;//5.510;
  float kPhiZeroAngleRight = 1.34986;//1.20721;
};


}
}




#endif /* VLOTTER_HPP */

