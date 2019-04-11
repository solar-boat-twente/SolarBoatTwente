/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   canbus.h
 * Author: Sander Oosterveld
 *
 * Created on April 11, 2019, 2:57 PM
 */

#ifndef CANBUS_H
#define CANBUS_H

#include <vector>
#include "../../include/can4linux.h"
#include <stdio.h>

namespace MIO{
struct CanStatus{

  unsigned int baudrate;

  char device[20];

  unsigned int buffer_left;

  unsigned int total_buffer;

  unsigned int double_reads;   

  bool status;
};


class CANbus{
  struct m_canmsg_t{
    // The can message
    canmsg_t msg;
    
    // Flag whether the message has been read before..
    // TRUE means it has not yet been read.
    bool FIRST_READER;    
  };
  

 public:
  CANbus(char device_name[20], unsigned int buffer_size, unsigned int baudrate=250000);
  /*
   * Initializes the canbus and starts it.
   * 
   * Arguments:
   *  char device_name[20]: ex. "can0", "can1"
   *  buffer_size: The max number of messages stored temporarily 
   */
      
      
      
  int read(unsigned long msg_id, canmsg_t *msg);
  /*
   * Looks in received messages if there is a message with that id.
   * If that it is the case writes that message to msg. 
   * 
   * Returns:
   *  -1 when failure (no msg id in list of msg_ids)
   *  0 when the message has been read before
   *  1 when successfull read
   */
  
  int write( canmsg_t* const msg);
  /*
   * Writes a can message to the canbus;
   */
  int start();
  /*
   * Starts the CANbus, it will open the device and stuff
   * 
   */
  int stop();
  /*
   * Closes the CANbus,
   */
  
  CanStatus status();
  /*
   * Gets the status of the Canbus
   */
  int add_message_(canmsg_t * const message);
  
 private:
  
  int _read_CAN();
  
  int _add_message(canmsg_t * const message);
  
  int _copy_message(canmsg_t *rx, canmsg_t *message);
  
  
  std::vector<CANbus::m_canmsg_t> received_message;
  
  CanStatus can_status;
  
};


}


#endif /* CANBUS_H */

