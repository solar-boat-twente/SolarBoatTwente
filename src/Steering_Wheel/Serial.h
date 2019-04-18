/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Serial.h
 * Author: Sander Oosterveld
 *
 * Created on April 16, 2019, 12:29 PM
 */

#ifndef SERIAL_H
#define SERIAL_H

#include <fcntl.h>
#include <termios.h>


namespace MIO{
/*
 * Class which handles reading the serial port
 * Example use:
 *  
 *  Serial * m_serial = new Serial(port);
 *  //TODO (sander); Implement the bottom three things
 *  // tty_old = m_serial.get_tty();
 *  // m_serial.tty_set(tty file);
 *  m_serial.start();
 *  m_serial.read(buf, max_bytes);
 *  m_serial.read_stop(buf, stop, max = 500);
 *  m_serial.close();
 * 
 * 
 * 
 */
class Serial {
 public:
  struct SerialStatus{
    
    char port[20];
    
    bool state;
    
    int baudrate;
    
    termios * tty = new termios;
    
  };
 public:
  /**
   * Opens the serial port with the defined port name and baudrate. 
   * Also applies standard setting.
   * 
   * @param port port name for the serial example: "/dev/ttyUSB0"
   * @param baudrate speed of the port in kbps, standard is 9600
   */
  Serial(const char port[20], int baudrate = 9600);
   
  /**
   * Destructor for the Serial port, closes the serial port mostly
   */
  ~Serial();
      
  /**
   * Closes the Serial port
   * @return result of the close function 1 for success, -1 for failure
   */
  int close();
  
  /**
   * Read a set number of bytes from the serial port
   * @param buf Stores the resulting bytes. 
   * @param max_bytes The number of bytes you want to read
   * @return 
   */
  int read_bytes(uint8_t buf[], short int max_bytes);

  /**
   * Reads the serial port until a certain stop byte
   * 
   * @param buf buffer in which the data will be written
   * @param stop A string where the byte should stop
   * @param stop_length The length of this stop string
   * @param max_bytes The maximum number of bytes before it stops looking standard 500
   * @return 
   */
  int read_stop(uint8_t buf[], const char stop[],int stop_length, short int max_bytes = 500);
  
 private:
  int file_descriptor;
  
  const int O_FLAGS = O_RDWR | O_NOCTTY;
  
  /**
   * For now makes adapts the termios structure to a standard usb interface.
   * @todo Make it possible to add custom settings.
   * @param tty, pointer to a termios struct gotten from Serial interface 
   * @return  1 for success, -1 for error. 
   */
  int make_settings_(termios * tty);
  
  /**
  * Applies the made settings, the setting can be made using make_settings_
  * @return 1 for success, -1 for error.
  * 
 */
  int apply_settings_();
  
  SerialStatus * const serial_status = new SerialStatus; 
};
}

#endif /* SERIAL_H */

