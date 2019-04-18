/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "canbus.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>

#include "../../include/easy_debugging.hpp"

using namespace MIO;
using namespace std;
/*public: */
CANbus::CANbus(char const device_name[], unsigned int buffer_size, unsigned int baudrate) {
  //Filling the can_status structure
  char device[20] = "/dev/";
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

int CANbus::open_can(int flag) {
  file_descriptor = open(can_status->device, flag);
  if(file_descriptor<0){
    M_ERR<<"UNABLE TO OPEN CAN DEVICE: "<<can_status->device;
    return -1;
  } else{
    M_OK<<"CANBUS WAS OPENED";
    return 1;
  }
}

int CANbus::close_can() {
  M_INFO<<"CLOSING CAN";
  return close(file_descriptor);
}

int CANbus::read_can(unsigned long msg_id, canmsg_t* buffer){
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

int CANbus::write_can(canmsg_t * const msg, bool force_send){
  canmsg_t msg_array[1];
  msg_array[0] = *msg;
  return write(file_descriptor, &msg_array, 1);
  
//  M_DEBUG<<"WRITTEN MESSAGE WITH ID: "<<msg->id<<" AND DATA: ";
//  for(int i = 0; i<msg->length;i++){
//    cout<<+msg->data[i]<<" ";
//  }
//  return 1;
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


CANbus::CanStatus * CANbus::status(){
  return can_status;
}


int CANbus::add_message_(canmsg_t* const message){
  canmsg_t * new_message = new canmsg_t;
  *new_message = *message;
  _add_message(new_message);
}

int CANbus::_read_CAN(){
  //Fakes making a simple message with id STD_ID and data STD_DATA
  M_INFO<<"Trying to read CAN";
  canmsg_t * rx = new canmsg_t;
  int got = read(file_descriptor, rx, can_status->total_buffer);
  M_INFO<<"GOT "<<got<<" MESSAGE";
  
  for(int i = 0; i<got; i++){
    _add_message(rx);
  }
  
  return got;
  /*
  canmsg_t *rx = new canmsg_t;
  rx->length = STANDARD_LENGTH;
  for(int i =0; i<STANDARD_LENGTH; i++){
    rx->data[i] = STANDARD_MESSAGE[i];
  }
  rx->id = STD_ID;
  M_INFO<<"CONSTRUCTED MESSAGE WITH DATA: "<<rx->data<<" AND ID: "<<rx->id;
  return _add_message(rx);
   * */
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

int MIO::print_canmsg(canmsg_t * const msg){
  int id = msg->id;
  short int length = msg->length;
  M_DEBUG<<"CAN_MESSAGE RECEIVED | ID: "<<id<<" LENGTH: "<<msg->length;
  std::cout<<"DATA: ";
  for(int i = 0; i<length; i++){
    std::cout<<(int)msg->data[i]<<" ";
  }
  std::cout<<std::endl;
  return 0;
}