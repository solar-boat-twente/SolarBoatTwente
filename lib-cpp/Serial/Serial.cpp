/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */


#include "../Debugging/easy_debugging.hpp"

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <iostream>


#include "Serial.h"

using namespace MIO;
using namespace std;

Serial::Serial(const char port[], int baudrate) {
  file_descriptor = open(port, O_FLAGS);
  
  
  //get the termios structure from the file_descriptor
  if(tcgetattr(file_descriptor, serial_status->tty)!=0){
    M_ERR<<errno<<" from tcgetattr: "<<strerror(errno);
  } else {
    make_settings_(serial_status->tty);
    apply_settings_();
    M_INFO<<"OPENED SERIAL DEVICE WITH FILE DESCRIPTOR: "<<file_descriptor;
    serial_status->status = true;
  }
}

Serial::~Serial(){
  M_WARN<<"CLOSING THE SERIAL PORT";
  serial_status->status = false;
}

int MIO::Serial::read_bytes(uint8_t buf[], short int max_bytes) {
  uint8_t m_buffer_ = '\0';
  int n_read = 0;
  
  if(max_bytes>1){
    M_INFO<<"READING MESSAGE:";
  }
  
  for(int i = 0; i<max_bytes; i++){
    n_read = read(file_descriptor, &m_buffer_, 1);
    buf[i]=m_buffer_;
    
    if(max_bytes>1){
      cout<<BOLD<<"\b\b\b\b\b\b\b\b\b\b\b\b\b\b"<<flush<<"MESSAGE: "<<i<<"/"<<max_bytes;
    }
    
    if (n_read<0){
      M_ERR<<"ERROR READING: "<<strerror(errno);
      break;
    }
  }
  
  if(max_bytes>1){
    cout<<RST<<endl;
  }
  
  if(n_read>1){
    M_DEBUG << "SERIAL RESPONSE: ";
    for(int i = 0; i<max_bytes; i++){
      std::cout<<BOLD<<showbase<<" "<<(int)buf[i]<<dec;
    } 
    std::cout<<RST<<endl;
    return max_bytes;
  } else if (n_read = 1){
    M_DEBUG<<"SERIAL RESPONSE: "<<BOLD<<showbase<<(int)buf[0]<<dec;
    return max_bytes;
  } else if (n_read<0){
    return -1;
  } else{
    M_WARN<<"NOT EVERYTHING WAS READ";
  }
}

int Serial::read_stop(uint8_t buf[], const char stop[], int stop_length, short int max_bytes) {
  uint8_t temp_buffer[max_bytes];
  uint8_t read_byte[1];
  for(int i = 0; i<max_bytes; i++){
    if(read_bytes(read_byte, 1)==1){
      temp_buffer[i]=read_byte[0];
    } else{
      return -1;
    }
    for(int j = 0; j<stop_length; j++){
      //printf("i = %i, BUFFER : %i, STOP: %i\n",i,temp_buffer[i], stop[j]);

      if(temp_buffer[i-(stop_length-j-1)]!=stop[j]){
        goto searching;
      } else{
        M_OK<<"PASSED ROUND: "<<j+1<<"/"<<stop_length<<" OF THE CHECK";
      }
    }
    for(int k =0; k<i-(stop_length-1); k++){
      buf[k]= temp_buffer[k];
    }
    return i-(stop_length-1);
    
    //Continue searching for the 
    searching: ;   
  }
  M_WARN<<"DID NOT REACH THE STOP BYTES";
  return -1;
}

int Serial::write_byte(uint8_t byte) {
  uint8_t bytes[1] = {byte};
  write(file_descriptor, bytes, 1);
  M_OK<<"WRITTEN A MESSAGE";
}


int Serial::make_settings_(termios * tty) {
    cfsetospeed(tty, (speed_t)B9600);
    cfsetispeed(tty, (speed_t)B9600);

    tty->c_cflag     &=  ~PARENB;            // Make 8n1
    tty->c_cflag     &=  ~CSTOPB;
    tty->c_cflag     &=  ~CSIZE;
    tty->c_cflag     |=  CS8;

    tty->c_cflag     &=  ~CRTSCTS;           // no flow control
    tty->c_cc[VMIN]   =  1;                  // read doesn't block
    tty->c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty->c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    // Make raw
    cfmakeraw(tty);
    
    //serial_status->tty = tty;
    return 1;
}

int Serial::apply_settings_() {
  tcflush(file_descriptor, TCIFLUSH);
  if ( tcsetattr ( file_descriptor, TCSANOW, serial_status->tty ) != 0) {
    M_ERR<< errno << " FROM TCSETATTR";
    return -1;
  } else {
    return 1;
  }
 }
