/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ADAM.hpp
 * Author: Sander Oosterveld
 *
 * Created on May 14, 2019, 1:12 PM
 */



#ifndef ADAM_HPP
#define ADAM_HPP

#include <modbus.h>



namespace MIO{
namespace UI{

const int COUNTER_READ_START = 40000;

class ADAM{

 public:

  /**
   * 
   * @param ip String with the ip of the ADAM 
   * @param port_number Port number
   * @param debug
   */
  ADAM(const char * ip, int port_number, bool debug = false);

  /**
  * Read out the current level of a pin, if the value is bigger than 17 it will read the coil state of DO
   * 
  * @param pin_number the number of the pin you want to read
  * @return if port is high or low
  */
  int read_port(int port_number);

  /**
   * Switches the state of a port value between 17 and 23 (port numbers of DO ports)
   * 
   * @param port_value
   * @param port_number
   * @return 
   */
  int write_port(int port_number, bool state);

  
  /**
   * 
   * @param port_number the port number you would like to read
   * @return integer the counter of the port number
   */
  int read_counter(int port_number);
  
  
  
 private:
  modbus_t* ctx;


};
}
}

#endif /* ADAM_HPP */

