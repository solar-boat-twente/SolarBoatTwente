/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <cstring>
#include <math.h>
#include <complex>

#include "Vlotter.hpp"
#include "../../../lib-cpp/Debugging/easy_debugging.hpp"

namespace MIO{
namespace control{



Vlotter::Vlotter(Serial * const serial) : serial_(serial){
  start_reading();
}

Vlotter::Vlotter() : serial_(new Serial(kVlotterSerialPort)) {
  start_reading();
}


Vlotter::~Vlotter() {
  stop_reading();
  delete serial_;
}

void Vlotter::configure(float new_phi_left, float new_phi_right) {
  kPhiZeroAngleLeft = new_phi_left;
  kPhiZeroAngleRight = new_phi_right;
}

void control::Vlotter::start_reading(short int delay) {
  if(serial_->get_status().status){
    if(!thread_active_){
      thread_active_ = true;
      M_OK<<"VLOTTER WAS STARTED ( ◞･౪･)";
      thread_ = std::thread(&Vlotter::reading_thread_, this, delay);
    } else{
      M_WARN<<"VLOTTER READING WAS ALREADY STARTED (¬_¬)";
    }
  } else {
      M_ERR<<"CANNOT START VLOTTER READING BECAUSE SERIAL NOT OPEN (╯•﹏•╰)";
  }  
}

void control::Vlotter::stop_reading() {
  if(thread_active_){
    thread_active_ = false;
    thread_.join();
    M_OK<<"VLOTTER HAS STOPPED READING";
  } else {
    M_WARN<<"VLOTTER READING WAS ALREADY STOPEED (¬_¬)";
 }
}


float control::Vlotter::get_angle_rad(EncoderNumber encoder) {
  if(encoder == EncoderNumber::ENCODER_LEFT){
    return angle_left_;
  } else if (encoder == EncoderNumber::ENCODER_RIGHT){
    std::cout<<"angle right: "<<angle_right_<<std::endl<<std::endl;
    return angle_right_;
  }
}

float control::Vlotter::get_height(EncoderNumber encoder) {
  if (encoder == EncoderNumber::ENCODER_RIGHT){
    return height_left_;
  } else {
    return height_right_;
  }
}

float Vlotter::get_roll_rad() {
  return roll_;
}


void control::Vlotter::reading_thread_(short int delay) {
  uint8_t bytes[4];
  int encoder_left;
  int encoder_right;
  while(thread_active_){
    read_vlotter_data_(bytes);
    
    encoder_left = bytes_to_int_(&(bytes[0]));
    encoder_right = bytes_to_int_(&(bytes[2]));
    
    angle_left_ = compute_angle_(encoder_left);
    height_left_ = compute_height_(kPhiZeroAngleLeft-angle_left_);
    
    angle_right_ = compute_angle_(encoder_right);
    height_right_ = compute_height_(kPhiZeroAngleRight-angle_right_);
      
    roll_ = compute_roll_(height_left_, height_right_);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    
  }


}


void control::Vlotter::read_vlotter_data_(uint8_t buffer[]) {
  uint8_t read_bytes[8];
  int receive_result = serial_->read_stop(read_bytes, kVlotterStopBytes, kVlotterStopLength, 8);
  if(receive_result==kVlotterMsgLength){
    memcpy(buffer, read_bytes, 4);    
  } else if (receive_result > 0){
    M_ERR<<"CONTROL WHEEL MESSAGE SIZE NOT CORRECT; SIZE EQUAL TO: "<<receive_result<<" (　ﾟдﾟ)";
  } else {
  }
}

int control::Vlotter::bytes_to_int_(uint8_t * bytes) {
  int msb = bytes[0]<<4;
  int lsb = bytes[1]>>4;
  
  return msb+lsb;
}

/**
 * returns the angle in degrees.
 * @param value value between 4096 and 0 gotten from the encoder
 * @return 
 */
float control::Vlotter::compute_angle_(int value) {
  return (float)value/4096 * 2*3.1415;
}
/**
 * NEEDS TO BE CORRECTED TO GET THE REAL HEIGHT!!
 * 
 * @param angle
 * @return 
 */
float control::Vlotter::compute_height_(float angle) {
  std::cout<<"Calculating height using angle: "<<angle<<std::endl;
  float height = kVlotterBeamLength * std::cos(angle);
  return height;  
}

float Vlotter::compute_roll_(float height_left, float height_right) {
  float roll = asin((height_left-height_right))*kVlotterDistance;
  return roll;
}

}
}