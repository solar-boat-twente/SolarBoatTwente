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
#include <thread>
#include <fcntl.h>

#include "../../include-cpp/can4linux.h"

namespace MIO {

constexpr int kStandardCanDelay  = 0; //delay between can searches in ms
constexpr int  kStandardCanBaudrate = 250; // standard baudrate
constexpr int  kStandardCanBuffer = 1;
constexpr int kStandardCanFlag = O_RDWR;


/*
 * Class wrapping all the CAN data for one line. 
Example:
   *  CANbus * const canbus = new canbus("can0", 100);
   *  canbus->start;
   *  canmsg_t receive_message;
   *  canbus->read(12458, receive_message);
   * 
   *  canmsg_t transmit_message;
   *  transmit_message.data[0] = 'a';
   *  transmit_message.data[1] = 'b'; \\etcetera
   *  transmit_message.id = 12548;
   *  transmit_message.length = 2;
   *  canbus->write(transmit_message);
   * 
   *  canbus->close(); 
 */
class CANbus {
  
 public:
  struct CanStatus {
    // Baudrate of the CANbus in bits so 250kb/s is 250
    uint16_t baudrate;

    // Name of the device ex: "can0", "can1" 
    std::string device;

    // Amount of buffer left before max length of buffer is gone
    // TODO(Sander): Implement the use of this
    uint16_t buffer_left;

    // Amount the total buffer was initially set to
    // TODO(Sander): Implemenent the use of this
    uint16_t total_buffer;

    // Amount of times something was read two time without needing to.
    // TODO(Sander): Implement the use of this
    uint16_t double_reads;

    // True if the CANbus is currently running, false otherwise
    bool status;
  
    // The standard file descriptor
    uint16_t file_descriptor = -1;
  
    // True if the canbus has been succesfully opened
    bool open;
  };
 
  struct m_canmsg_t {
    // The can message
    canmsg_t msg;
    // Flag whether the message has been read before..
    // TRUE means it has not yet been read.
    bool FIRST_READER;
  };


 public:

  /*
   * Initializes the canbus and starts it.
   * 
   * Arguments:
   *  char device_name[20]: ex. "can0", "can1"
   *  buffer_size: The max number of messages stored temporarily 
   * 
   * 
   *  
   */
  CANbus(const std::string& device_name, unsigned int buffer_size = kStandardCanBuffer, unsigned int baudrate = kStandardCanBaudrate, int flag = kStandardCanFlag);
  
  virtual ~CANbus();
  
  int open_can(int flag = kStandardCanFlag);
  
  int close_can();
  
  /*
   * Looks in received messages if there is a message with that id.
   * If that it is the case writes that message to msg. 
   * 
   * Arguments:
   *  unsigned long msg_id: The message id you want to have the data from
   *  canmsg_t *buffer: A pointer to the buffer where the message will be written
   * 
   * Returns :
   *  -1: when failure (no msg id in list of msg_ids)
   *  0: when the message has been read before
   *  1: when read
   */
  int read_can(unsigned long msg_id, canmsg_t * const buffer);


  /*
   * Writes a can message to the canbus;
   * 
   * Arguments:
   *  cansmsg_t * const message: A can structure of the message you want to send
   *  force_send: boolean to set if a certain thing has to be send continiously
   * 
   * Returns:
   *  -1: Failure
   *  0: Nothing written
   *  1: Success
   */
  int write_can(const canmsg_t& msg, bool force_send = false);


  /*
   * Starts the CANbus, it will open the device and stuff
   * 
   * Returns:
   *  -1: Start unsuccessful
   *  1: Success
   */
  int start(short int delay = kStandardCanDelay);


  /*
   * Closes the CANbus,
   * 
   * Returns:
   *  -1: Close unsuccessful
   *  1: Success
   */
  int stop();


  /*
   * Gets the status of the Canbus
   */
  CanStatus& get_status();

   /**
   * Uses IOCTL
   * 
   * @param baudrate
   * @return 
   */
  int set_baudrate(unsigned int baudrate);
  
 private:

  int _read_CAN();

  int add_message_(const canmsg_t &rx);

  int copy_message_(canmsg_t *rx, canmsg_t *message);

  void read_can_thread_(short int delay=kStandardCanDelay);
  
  std::vector<CANbus::m_canmsg_t> received_message;

  CanStatus can_status;  
  
  int file_descriptor = -1;
  
  std::thread m_thread;
};

int print_canmsg(canmsg_t * const msg);



}


#endif /* CANBUS_H */
