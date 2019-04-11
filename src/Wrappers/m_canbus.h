/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   m_canbus.h
 * Author: Sander Oosterveld
 *
 * Wraps the CANbus using 
 * 
 * Created on April 9, 2019, 7:03 PM
 */

//TODO: implement these functions, fix the imports.

#ifndef M_CANBUS_H
#define M_CANBUS_H

#include <vector>
#include "../../include/can4linux.h"
#include "stdio.h"

namespace top_level{

namespace canbus{

struct m_canmsg_t{
  canmsg_t msg;
  bool FLAG = false;
};

struct power_canmsg_t{
  std::vector<m_canmsg_t> bms_msg;
  
  std::vector<m_canmsg_t> driver_msg;
  
  std::vector<m_canmsg_t> mppt_msg;
};

class CAN{
 public:
  CANbusWrapper(char device_name[20], unsigned int baudrate=250000, unsigned int message_length = 500, bool blocking = true);
  /*Initiate with the device name
   * 
   * Inputs:
   * char[20] device name e.g. "can0" or "can1"
   * unsigned int baudrate (standard 250kbs (250000))
   * unsigned int message length: maximum amount of messages for receiving (standard = 500)
   * bool blocking: Sets whether it should block or not (standard = true)
   */
  int write(unsigned long msg_id, unsigned char *msg_data, int msg_length);
  /*Writes one message through the CAN bus
   *
   * Parameters:
   *  unsigned long msg_id: The message id of the send message
   *  unsigned char *msg_data: a pointer to a byte array of msg_data
   *  int msg_length: The message length
   *  
   * Returns:
   *  0 for success, 1 for failure.
   */
  int read_all(std::vector<canmsg_t> *data);
  /*Read all the data once and stores it into an array of canmsg_t of length message_length
   * 
   * Parameters:
   *  canmsg_t *data: Pointer to a vector of canmsg_t where the messages will be written
   * 
   * Returns:
   *  -1 for failure, else the number of messages received
   */
  
  int read_filtered(unsigned long msg_id, canmsg_t *data);
  /*Reads all the data but only returns the filtered ones inside a data array. Be very careful since all other data is lost.
   * 
   * Parameters:
   *  unsigned long msg_id: The msg_id to filter on.
   *  canmsg_t *data: Pointer to an array of canmsg_t's to store the filtered messages
   * 
   * Returns:
   *  -1 for failure/error, else the number of messages received.
   * 
   */
  int close_can();
  /*
   * Closes the file stream 
   * 
   * Returns:
   *  0 for success, 1 for error/failure
   */
  
  int open_can();
  /* Opens the can port defined when constructing this class, is already called in constructor
   * 
   * Returns:
   *  0 for success, 1 for error/failure
   */
  
  int set_baud(unsigned int baudrate);
   /* Sets the baud rate
    * 
    * Parameters:
    *   unsigned int baudrate: The new baudrate in bps
    * Returns:
    *   0 for success, 1 for error/failure
    */
  
  int set_blocking(bool blocking);
  /*Sets whether the can_connection should block
   * 
   * Parameters:
   *  bool blocking: Sets the blocking variable
   * 
   * Returns:
   *  0 for success, 1 for failure/error
   */
  
  int parse_data(std::vector<canmsg_t> *messages);
  
  power_canmsg_t power;
  
 private:  
  int fd;
  char DEVICE[20]; // The device name
  unsigned int BAUD; // Baudrate
  unsigned int MAX; //Max number of received messages
  bool BLOCK; //Blocking on or off.
  
  
  
};


}
}

#endif /* M_CANBUS_H */

