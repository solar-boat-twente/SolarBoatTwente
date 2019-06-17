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
namespace Control{
void Control::Vlotter::start_reading(short int delay) {
  if(serial_->status()->status){
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

float Control::Vlotter::get_angle_deg(EncoderNumber encoder) {
  if(encoder == ENCODER_LEFT){
    return angle_left_;
  } else if (encoder == ENCODER_RIGHT){
    std::cout<<"angle right: "<<angle_right_<<std::endl<<std::endl;
    return angle_right_;
  }
}

float Control::Vlotter::get_height_deg(EncoderNumber encoder) {
  if (encoder == ENCODER_RIGHT){
    return height_left_;
  } else {
    return height_right_;
  }
 }


void Control::Vlotter::reading_thread_(short int delay) {
  uint8_t bytes[4];
  int encoder_left;
  int encoder_right;
  while(thread_active_){
    read_vlotter_data_(bytes);
    encoder_left = bytes_to_int_(&(bytes[0]));
    encoder_right = bytes_to_int_(&(bytes[2]));
    
    angle_left_ = calculate_angle_(encoder_left);
    height_left_ = calculate_height_(angle_left_);
    
    angle_right_ = calculate_angle_(encoder_right);
    height_right_ = calculate_height_(angle_right_);
      
    
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    
  }


}


void Control::Vlotter::read_vlotter_data_(uint8_t buffer[]) {
  uint8_t read_bytes[8];
  int receive_result = serial_->read_stop(read_bytes, VLOTTER_STOP_BYTES, VLOTTER_STOP_LENGTH, 8);
  if(receive_result==VLOTTER_MSG_LENGTH){
    memcpy(buffer, read_bytes, 4);    
  } else if (receive_result > 0){
    M_ERR<<"CONTROL WHEEL MESSAGE SIZE NOT CORRECT; SIZE EQUAL TO: "<<receive_result<<" (　ﾟдﾟ)";
  } else {
  }
}

int Control::Vlotter::bytes_to_int_(uint8_t * bytes) {
  int msb = bytes[0]<<4;
  int lsb = bytes[1]>>4;
  
  return msb+lsb;
}

/**
 * returns the angle in degrees.
 * @param value value between 4096 and 0 gotten from the encoder
 * @return 
 */
float Control::Vlotter::calculate_angle_(int value) {
  return (float)value/4096 * 2*3.1415;
}
/**
 * NEEDS TO BE CORRECTED TO GET THE REAL HEIGHT!!
 * 
 * @param angle
 * @return 
 */
float Control::Vlotter::calculate_height_(float angle) {
  return std::sin(angle/180 * 3.1415) * VLOTTER_BEAM_LENGTH;
 }






}
}