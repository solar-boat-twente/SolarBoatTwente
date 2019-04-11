/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "canbus.h"
#include <iostream>
using namespace MIO;
CANbus::CANbus(char device_name[], unsigned int buffer_size, unsigned int baudrate) {
  printf("Initializing CAN device with name: /dev/%s\n", device_name);
  _read_CAN();
 }

int CANbus::read(unsigned long msg_id, canmsg_t* msg){
  for(CANbus::m_canmsg_t &message: received_message){
    std::cout<<message.msg.id<<" "<<message.FIRST_READER<<" "<<message.msg.data<<std::endl;
  }
  for(CANbus::m_canmsg_t &message: received_message){
    if (message.msg.id == msg_id){
      *msg = message.msg;
      if(message.FIRST_READER){
        message.FIRST_READER = false;
        return 1;
      } else{
        return 0;
      }
    }
  }
  return -1;
}

int CANbus::write(canmsg_t* const msg){
  std::cout<<"written a message to can with data: "<<msg->data<<" and id: "<<msg->id<<std::endl;
} 


int CANbus::_read_CAN(){
  canmsg_t rx;
  rx.data[0] = 'a';
  rx.data[1] = 'a';
  rx.data[2] = 'a';
  rx.length = 3;
  rx.id = 12548;
  printf("Constructed message with data: %s and id: %i\n", rx.data, (int)rx.id);
  return _add_message(&rx);
  
  
}

int CANbus::_add_message(canmsg_t* rx){
  for(m_canmsg_t &message: received_message){
    if(message.msg.id == rx->id){
      message.msg = *rx;
      message.FIRST_READER = true;
      printf("Updated message with data: %s and id: %i\n", message.msg.data, (int)message.msg.id);
      return 0;
    }
  }
  received_message.push_back(CANbus::m_canmsg_t());
  received_message[received_message.size()-1].FIRST_READER = true;
  received_message[received_message.size()-1].msg= *rx;
  return 0;
}

int CANbus::add_message_(canmsg_t* const message){
  _add_message(message);
}
