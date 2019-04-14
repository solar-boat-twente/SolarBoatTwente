/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "canbus.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>

#include "../../include/easy_debugging.hpp"

using namespace MIO;
using namespace std;
/*public: */
CANbus::CANbus(char device_name[], unsigned int buffer_size, unsigned int baudrate) {
  //Filling the can_status structure
  char device[20];
  int len = sprintf(device, "/dev/%s", device_name);
  for(int i=0; i<len;i++){
    can_status->device[i] = device[i];
  }
  can_status->total_buffer = buffer_size;
  can_status->baudrate = baudrate;
  can_status->buffer_left = buffer_size;
  can_status->double_reads = 0;
  can_status->status = false;
  
  //Starting the background thread
  //start();
    
 }

int CANbus::read(unsigned long msg_id, canmsg_t* buffer){
  for(CANbus::m_canmsg_t &message: received_message){
    if (message.msg->id == msg_id){
      *buffer = *message.msg;
      if(message.FIRST_READER){
        message.FIRST_READER = false;
        M_OK<<"SUCCESSFULLY READ MESSAGE WITH ID: "<<msg_id;
        return 1;
      } else{
        M_WARN<<"TRIED TO READ MESSAGE WITH ID: "<<msg_id<<" TWICE";
        return 0;
      }
    }
  }
  M_ERR<<"INVALID CAN ID:"<<msg_id;
  return -1;
}

int CANbus::write(canmsg_t* const msg, bool force_send){
  M_DEBUG<<"WRITTEN MESSAGE WITH ID: "<<msg->id<<" AND DATA: ";
  for(int i = 0; i<msg->length;i++){
    cout<<+msg->data[i]<<" ";
  }
  return 1;
} 

//TODO(sander): Make sure that canbus cannot be started twice!
int CANbus::start(short int delay){
  //Making a thread 
  m_thread = std::thread(&CANbus::_read_CAN_thread, this, delay);
  can_status->status = true;
  M_INFO<<"CANBUS STARTED WITH DEVICE: "<<can_status->device;
  return 1;
}

//TODO(sander): Make sure that canbus cannot be stopped twice!
int CANbus::stop(){
  can_status->status = false;
  m_thread.join();
  M_INFO<<"CANBUS STARTED WITH DEVICE: "<<can_status->device;
  return 1;
}


CanStatus* CANbus::status(){
  return can_status;
}


int CANbus::add_message_(canmsg_t* const message){
  canmsg_t * new_message = new canmsg_t;
  *new_message = *message;
  _add_message(new_message);
}

int CANbus::_read_CAN(){
  //Fakes making a simple message with id STD_ID and data STD_DATA
  canmsg_t *rx = new canmsg_t;
  rx->length = STANDARD_LENGTH;
  for(int i =0; i<STANDARD_LENGTH; i++){
    rx->data[i] = STANDARD_MESSAGE[i];
  }
  rx->id = STD_ID;
  M_INFO<<"CONSTRUCTED MESSAGE WITH DATA: "<<rx->data<<" AND ID: "<<rx->id;
  return _add_message(rx);
}

/*Private:*/
int CANbus::_add_message(canmsg_t* rx){
  // Update old message
  for(m_canmsg_t &message: received_message){
    if(message.msg->id == rx->id){
      delete message.msg;
      message.msg = rx;
      message.FIRST_READER = true;
      M_INFO<<"UPDATED MESSAGE WITH DATA: "<<message.msg->data<<" AND ID: "<<rx->id;
      return 0;
    }
  }
  
  // Making a new message and adding that to all the received messages
  m_canmsg_t new_message;
  new_message.FIRST_READER = true;
  new_message.msg = rx;
  received_message.push_back(new_message);
  M_INFO<<"ADDED NEW MESSAGE TO VECTOR WITH ID: "<<new_message.msg->id;
  return 1;
}

void CANbus::_read_CAN_thread(short int delay){
  while(can_status->status){
    _read_CAN();    
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
   }
  
  
}
