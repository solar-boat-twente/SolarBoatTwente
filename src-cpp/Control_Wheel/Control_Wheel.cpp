/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */


#include "../../lib-cpp/Debugging/easy_debugging.hpp"

#include "Control_Wheel.hpp"

using namespace MIO;
using namespace UI;

using namespace std;

ControlWheel::ControlWheel(Serial* serial):serial_(serial) {
  control_wheel_status.serial_state = &serial->get_status();
  if (!control_wheel_status.serial_state->status){
    M_WARN<<"SERIAL NOT OPENED! ( ̵˃﹏˂̵ )";
  } else {
    M_OK<<"CONTROL WHEEL SUCCESSFULLY INITIALIZED ( ◞･౪･)";
  }
}

int ControlWheel::start_reading(structures::UserInput* user_input, short int delay) {
  if(control_wheel_status.serial_state->status){
    if(!control_wheel_status.read_state){
      control_wheel_status.read_state = true;
      M_OK<<"CONTROL WHEEL WAS STARTED ( ◞･౪･)";
      m_reading_thread_ = thread(&ControlWheel::reading_thread_, this, user_input, delay);
      return 1;
    } else{
      M_WARN<<"CONTROL WHEEL READING WAS ALREADY STARTED (¬_¬)";
      return -1;
    }
  } else {
      M_ERR<<"CANNOT START READING BECAUSE SERIAL OPEN (╯•﹏•╰)";
      return -1;
  }  
}

int ControlWheel::get_data_(structures::UserInput* user_input) {
  uint8_t read_bytes[8];
  int receive_result = serial_->read_stop(read_bytes, CONTROL_WHEEL_STOP_BYTES, CONTROL_WHEEL_STOP_LENGTH, 8);
  if(receive_result==CONTROL_WHEEL_MSG_LENGTH){
    std::vector<uint8_t> *data = new std::vector<uint8_t>;
    for(int i = 0; i<CONTROL_WHEEL_MSG_LENGTH;i++){
      data->push_back(read_bytes[i]);
    }
    parse_data_(data, user_input);
    delete data;
    
    return 1;
  } else if (receive_result > 0){
    M_ERR<<"CONTROL WHEEL MESSAGE SIZE NOT CORRECT; SIZE EQUAL TO: "<<receive_result<<" (　ﾟдﾟ)";
    return -1;      
  } else {
    return -1;
  }
  
  
  
}

int ControlWheel::parse_data_(std::vector<uint8_t>* bytes, structures::UserInput* user_input) {
  
    user_input->steer.reverse = (*bytes)[0];
    user_input->steer.raw_throttle = (*bytes)[2]*256+(*bytes)[3];
    
    switch ((*bytes)[1]){
      case 0:
        user_input->steer.fly_mode = structures::NO_FLY;
        break;
    };
    
    M_OK<<"READING AND PARSING CONTROL WHEEL RESPONSE SUCCESSFUL! d(>_･ )";
    M_DEBUG<<"RESULT FROM CONTROL WHEEL IS: \n"<<" Reverse: "<<(int)user_input->steer.reverse<<
        " | Fly Mode: "<<user_input->steer.fly_mode<<" | Raw Throttle: "<<user_input->steer.raw_throttle<<"\n";
    return 1;
}

void ControlWheel::reading_thread_(structures::UserInput* user_input, short int delay) {
  M_INFO<<"CONTROL WHEEL READING THREAD HAS STARTED WITH DELAY: "<<(long)delay<<"ms ( ͡° ͜ʖ ͡°)";
  while(control_wheel_status.read_state){
    get_data_(user_input);
    std::this_thread::sleep_for(chrono::milliseconds(delay));
  }
  M_INFO<<"READING THREAD WAS STOPPED";
  
}
  
 

