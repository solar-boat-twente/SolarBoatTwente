/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Button_Encoder.cpp
 * Author: Yvon
 *
 * Created on May 1, 2019, 2:02 PM
 */

#include "../../lib-cpp/Debugging/easy_debugging.hpp"
#include <errno.h>
#include "Encoders.hpp"

using namespace MIO;
using namespace UI;
using namespace structures;
using namespace std;

ButtonEncoder::ButtonEncoder(Serial* serial):serial_(serial) {
  button_encoder_status->serial_state = serial->status();
  if (!button_encoder_status->serial_state->status){
    M_WARN<<"SERIAL NOT OPENED!";
  } else {
    M_OK<<"BUTTON ENCODER SUCCESSFULLY INITIALIZED ";
  }
}

int ButtonEncoder::start_reading(structures::UserInput* user_input, short int delay) {
  if(button_encoder_status->serial_state->status){
    if(!button_encoder_status->read_state){
      button_encoder_status->read_state = true;
      M_OK<<"BUTTON ENCODER WAS STARTED";
      m_reading_thread_ = thread(&ButtonEncoder::reading_thread_, this, user_input, delay);
      return 1;
    } else{
      M_WARN<<"BUTTON ENCODER READING WAS ALREADY STARTED ";
      return -1;
    }
  } else {
      M_ERR<<"CANNOT START READING BECAUSE SERIAL OPEN ";
      return -1;
  }  
}

int ButtonEncoder::get_data_(structures::UserInput* user_input) {
  uint8_t read_bytes[8];
  int receive_result = serial_->read_stop(read_bytes, BUTTON_STOP_BYTES, BUTTON_STOP_LENGTH, 8);
  if(receive_result==BUTTON_MSG_LENGTH){
    std::vector<uint8_t> *data = new std::vector<uint8_t>;
    for(int i = 0; i<BUTTON_MSG_LENGTH;i++){
      data->push_back(read_bytes[i]);
    }
    parse_data_(data, user_input);
    delete data;
    
    return 1;
  } else if (receive_result > 0){
    M_ERR<<"BUTTON ENCODER MESSAGE SIZE NOT CORRECT; SIZE EQUAL TO: "<<receive_result<<" ";
    return -1;      
  } else {
    return -1;
  }
  
  
  
}

int ButtonEncoder::parse_data_(std::vector<uint8_t>* bytes, structures::UserInput* user_input) {
 
    // parse data depending on state of boat from Button Encoder 1
    switch ((*bytes)[0]*256){
        case 1:
            user_input->control.PID_roll = structures::STATE1;
            break;
        case 2:
            user_input->control.PID_roll = structures::STATE2;
            break;
        case 3:
             user_input->control.PID_roll = structures::STATE3;
             break;
        case 4:
             user_input->control.PID_roll = structures::STATE4;
             break;
        case 5:
             user_input->control.PID_roll = structures::STATE5;
             break;
        case 6:
             user_input->control.PID_roll = structures::STATE6;
             break;
        case 7:
             user_input->control.PID_roll = structures::STATE7;
             break;
        case 8:
             user_input->control.PID_roll = structures::STATE8;
             break;
     };
    
       // parse data depending on state of boat from Button Encoder 2
    switch ((*bytes)[1]*256){
        case 1:
            user_input->control.PID_pitch = structures::STATE1;
            break;
        case 2:
            user_input->control.PID_pitch = structures::STATE2;
            break;
        case 3:
             user_input->control.PID_pitch = structures::STATE3;
             break;
        case 4:
             user_input->control.PID_pitch = structures::STATE4;
             break;
        case 5:
             user_input->control.PID_pitch = structures::STATE5;
             break;
        case 6:
             user_input->control.PID_pitch = structures::STATE6;
             break;
        case 7:
             user_input->control.PID_pitch = structures::STATE7;
             break;
        case 8:
             user_input->control.PID_pitch = structures::STATE8;
             break;
     };

       // parse data depending on state of boat from Button Encoder 3
    switch ((*bytes)[2]*256){
        case 1:
            user_input->control.PID_height = structures::STATE1;
            break;
        case 2:
            user_input->control.PID_height = structures::STATE2;
            break;
        case 3:
             user_input->control.PID_height = structures::STATE3;
             break;
        case 4:
             user_input->control.PID_height = structures::STATE4;
             break;
        case 5:
             user_input->control.PID_height = structures::STATE5;
             break;
        case 6:
             user_input->control.PID_height = structures::STATE6;
             break;
        case 7:
             user_input->control.PID_height = structures::STATE7;
             break;
        case 8:
             user_input->control.PID_height = structures::STATE8;
             break;
     };
    
    
    M_OK<<"READING AND PARSING BUTTON ENCODER RESPONSE SUCCESSFUL! ";
    M_DEBUG<<"RESULT FROM BUTTON ENCODER IS: \n"<<" Button 1: "<<user_input->control.PID_roll<<" | Button 2: "<<user_input->control.PID_pitch<<" | Button 3: "<<user_input->control.PID_height<<"\n";
    return 1;
}


void ButtonEncoder::reading_thread_(structures::UserInput* user_input, short int delay) {
  M_INFO<<"BUTTON ENCODER READING THREAD HAS STARTED WITH DELAY: "<<(long)delay<<"ms ";
  while(button_encoder_status->read_state){
    get_data_(user_input);
    std::this_thread::sleep_for(chrono::milliseconds(delay));
  }
  M_INFO<<"READING THREAD WAS STOPPED";
  
}
