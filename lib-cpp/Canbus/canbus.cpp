/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "canbus.h"

#include <sys/ioctl.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>


#include "../Debugging/easy_debugging.hpp"

using namespace MIO;
using namespace std;
/*public: */

CANbus::CANbus(const char * device_name, unsigned int buffer_size, unsigned int baudrate, int flag) {
  //Filling the can_status structure
  can_status->device = device_name;
  can_status->total_buffer = buffer_size;
  can_status->baudrate = baudrate;
  can_status->buffer_left = buffer_size;
  can_status->double_reads = 0;
  can_status->status = false;
  can_status->open = false;
  //Opens the CANbus
  if(open_can(flag)>0){
    set_baudrate(baudrate);
  };
  
  
  //Starting the background thread
  //start();
    
}

int CANbus::open_can(int flag) {
  //Tests if the canbus is not yet open. 
  if(!can_status->open){
    file_descriptor = open(can_status->device, flag);
    std::cout<<file_descriptor;
    if(file_descriptor<0){
      M_ERR<<"UNABLE TO OPEN CAN DEVICE: "<<can_status->device;
      return -1;
    } else{
      M_OK<<"CANBUS WAS OPENED (°‿↼)";
      can_status->open = true;
      return 1;
    }
  } else {
    M_WARN<<"CANBUS HAS ALREADY BEEN OPENED, CLOSE THE CANBUS BEFORE REOPENING (╯°□°）╯︵ ┻━┻";
    return -1;
  }
}

int CANbus::close_can() {
  if(can_status->open){
    M_INFO<<"CLOSING CAN (*＾▽＾)／";
    can_status->open = false;
    return close(file_descriptor);
  } else {
    M_WARN<<"CANBUS HAS NOT YET BEEN OPENED, DON'T CLOSE A CLOSED CONNECTION! (='_' )";
  }
}

int CANbus::read_can(unsigned long msg_id, canmsg_t* buffer){
  for(CANbus::m_canmsg_t &message: received_message){
    if (message.msg->id == msg_id){
      *buffer = *message.msg;
      if(message.FIRST_READER){
        message.FIRST_READER = false;
        M_OK<<"SUCCESSFULLY READ MESSAGE WITH ID: "<<(short int)msg_id<<" (｡♥‿♥｡)";
        return 1;
      } else{
        M_WARN<<"TRIED TO READ MESSAGE WITH ID: "<<(short int)msg_id<<" TWICE (-_-)";
        return 0;
      }
    }
  }
  M_ERR<<"INVALID CAN ID:"<<(short int)msg_id;
  return -1;
}

int CANbus::write_can(canmsg_t * const msg, bool force_send){
  if(can_status->open){
    canmsg_t msg_array[1];
    msg_array[0] = *msg;
    int success = write(file_descriptor, &msg_array, 1);
    if(success<0){
      M_ERR<<"ERROR WRITING: "<<strerror(errno);
    } else if(success == 0){
      M_WARN<<"NOTHING WAS WRITTEN ¯\\_(ツ)_/¯";
    } else {
      M_OK<<"WRITTEN A CAN MESSAGE WITH ID: "<<(short int)msg->id<<" ~(˘▾˘~) AND DATA: ";
      for(int i = 0; i<msg->length; i++){
        std::cout<<BOLD<<showbase<<hex<<" "<<(int)msg->data[i]; 
      }
      std::cout<<RST<<dec<<"\n"<<endl;
      
    }
    return success;
  } else {
    M_WARN<<"OPEN THE CANBUS BEFORE WRITING YOU FOOL! (╯°□°）╯︵ ┻━┻";
  }
//  M_DEBUG<<"WRITTEN MESSAGE WITH ID: "<<msg->id<<" AND DATA: ";
//  for(int i = 0; i<msg->length;i++){
//    cout<<+msg->data[i]<<" ";
//  }
//  return 1;
} 

//TODO(sander): Make sure that canbus cannot be started twice!
int CANbus::start(short int delay){
  //Making a thread 
  if(!can_status->status){
    m_thread = std::thread(&CANbus::_read_CAN_thread, this, delay);
    can_status->status = true;
    M_INFO<<"CANBUS STARTED WITH DEVICE: "<<can_status->device<<" (^～^)";
    return 1;
  } else {
    M_WARN<<"YOU ALREADY STARTED THE CANBUS, YOU FOOL! (╯°□°）╯︵ ┻━┻";
  }
}

//TODO(sander): Make sure that canbus cannot be stopped twice!
int CANbus::stop(){
  if(can_status->status){
    can_status->status = false;
    m_thread.join();
    M_INFO<<"CANBUS STOPPED FOR DEVICE: "<<can_status->device;
    return 1;
  } else {
    M_WARN<<"YOU HAVE TO START THE CANBUS FIRST... (⁀⊙﹏☉⁀ )";
  }
}


CANbus::CanStatus * CANbus::status(){
  return can_status;
}

int CANbus::set_baudrate(unsigned int baudrate) {
  Config_par_t cfg;
  volatile Command_par_t cmd;
  
  cmd.cmd = CMD_STOP;
  ioctl(file_descriptor, CAN_IOCTL_COMMAND, &cmd);
  
  cfg.target = CONF_TIMING;
  cfg.val1 = baudrate;
  ioctl(file_descriptor, CAN_IOCTL_CONFIG, &cfg);
  
  cmd.cmd = CMD_START;
  ioctl(file_descriptor, CAN_IOCTL_COMMAND, &cmd);
  
  M_INFO<<"CHANGED THE CAN BAUDRATE TO "<<baudrate<<" Kbps!";
  return 1;
 }



int CANbus::add_message_(canmsg_t* const message){
  canmsg_t * new_message = new canmsg_t;
  *new_message = *message;
  _add_message(new_message);
}

int CANbus::_read_CAN(){
  //Makes a new dynamic canmsg_t, this is deleted once the message is overwritten.
  canmsg_t * rx = new canmsg_t;
  int got = read(file_descriptor, rx, can_status->total_buffer);
  
  for(int i = 0; i<got; i++){
    _add_message(rx);
  }
  
  return got;
}

/*Private:*/
int CANbus::_add_message(canmsg_t* rx){
  // Update old message
  for(m_canmsg_t &message: received_message){
    if(message.msg->id == rx->id){
      delete message.msg;
      message.msg = rx;
      message.FIRST_READER = true;
      M_INFO<<"UPDATED MESSAGE WITH ID: "<<(short int)rx->id<<" AND DATA: ";
      for(int i = 0; i<rx->length; i++){
        std::cout<<BOLD<<showbase<<hex<<" "<<(int)rx->data[i]; 
      }
      std::cout<<RST<<dec<<"\n"<<endl;
      return 0;
    }
  }
  
  // Making a new message and adding that to all the received messages
  m_canmsg_t new_message;
  new_message.FIRST_READER = true;
  new_message.msg = rx;
  received_message.push_back(new_message);
  M_INFO<<"ADDED NEW MESSAGE TO VECTOR WITH ID: "<<(short int)new_message.msg->id<<" AND DATA: ";
  for(int i = 0; i<rx->length; i++){
    std::cout<<BOLD<<showbase<<hex<<" "<<(int)rx->data[i]; 
  }
  std::cout<<RST<<dec<<"\n"<<endl;
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