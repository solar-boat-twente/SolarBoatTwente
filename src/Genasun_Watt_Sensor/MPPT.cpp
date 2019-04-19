/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <cstdint>

#include "MPPT.h"
#include "../../include/easy_debugging.hpp"
#include "../../include/can4linux.h"

using namespace MIO;

using namespace MIO::PowerElectronics;

MPPT::MPPT(CANbus * const canbus, uint8_t number_of_mppts, unsigned long start_address, unsigned long relay_address) : number_of_mppts_(number_of_mppts) {
  start_address_ = start_address;
  relay_address_ = relay_address;
  
  if(canbus->status()->open && canbus->status()->status){
    canbus_ = canbus;
    M_OK<<"MPPT SUCCESFULLY INITIALIZED ^_^";
  } else{
    M_ERR<<"MAKE SURE YOU USE A PROPER CANBUS OBJECT WHICH IS RUNNING AND OPEN! (ー_ー)!!";
  }
}

int MPPT::get_bytes_from_address(uint8_t buf[], unsigned long address) {
  if(canbus_!=NULL){
    canmsg_t raw_receive;
    int success = canbus_->read_can(address, &raw_receive);
    if(success==1){
      if(raw_receive.length!=8){
        M_WARN<<"MESSAGE LENGTH NOT EQUAL TO 8, MAYBE INVALID ID!";
      }
      M_DEBUG<<"GOT THE FOLLOWING DATA: ";
      for(int i = 0; i<raw_receive.length; i++){
        buf[i] = raw_receive.data[i];
        std::cout<<(int)buf[i]<<" ";
      }
      std::cout<<std::endl;
    }
    return success;
  } else {
    M_WARN<<"WHAT DID I TELL YOU ABOUT THE CANBUS?!?!, RUN IT FIRST!";
    return -1;
  }
}


int MPPT::get_float_from_address(float buf[], unsigned long address) {
  uint8_t raw_result[8];
  int receive_success = get_bytes_from_address(raw_result, address)==1;
  if(receive_success==1){
    for(int i = 0; i<4; i++){
      //Results are a voltage for i = 0,2 and current for i = 1,3, change accordingly
      if(i%2){
        buf[i] = get_V_float(raw_result[2*i], raw_result[2*i + 1]);
      } else {
        buf[i] = get_A_float(raw_result[2*i], raw_result[2*i + 1]);
      }
    }
    return 1;
  } else {
    return receive_success;
  }

}


int MPPT::get_bytes_from_number(uint8_t buf[], int cell_number) {
  //Checks if the cell_number is valid, for now it allows it.
  if(cell_number<start_address_ or cell_number>start_address_+number_of_mppts_){
    M_WARN<<"CELL NUMBER NOT WITHIN RANGE OF MPPTS";
    //return -1;
  }
  
  //Gets the address from the cell_number
  unsigned long address = start_address_ + cell_number - 1;
  
  //Writes buf using the get_bytes_from_address
  uint8_t bytes_from_address[8];
  int success = get_bytes_from_address(bytes_from_address, address);
  for(int i = 0; i<8; i++){
    buf[i] = bytes_from_address[i];
  }
  return success;
}

int MPPT::get_float_from_number(float buf[], int cell_number) {
  //Checks if the cell_number is valid, for now it allows it.
  if(cell_number<start_address_ or cell_number>start_address_+number_of_mppts_){
    M_WARN<<"CELL NUMBER NOT WITHIN RANGE OF MPPTS";
    //return -1;
  }
  
  //Gets the address from the cell_number
  unsigned long address = start_address_ + cell_number - 1;
  
  float floats_from_address[8];
  int success = get_float_from_address(floats_from_address, address);
  for(int i = 0; i<8; i++){
    buf[i] = floats_from_address[i];
  }

  
  return get_float_from_address(buf, address);
}

int MPPT::get_all_bytes_data(uint8_t buf[][8]) {
  int received = 0;
  int success;
  uint8_t raw_result[8]; //Buffer to store the result for one address
  int count = 0; //Stores the row which the data is written to. 
  for(int address = start_address_; address<start_address_+number_of_mppts_; address++){
    //Gets the bytes for one address and writes it to the final matrix
    success = get_bytes_from_address(raw_result, address);
    
    for(int j = 0; j<8; j++){
      buf[count][j] = raw_result[j];
    }
    count++;
    
    if(success<0){
      return -1;
    } else {
      received+=success;
    }
  }
  return received;
}

int MPPT::get_all_float_data(float buf[][4]) {
  uint8_t raw_result[number_of_mppts_][8];
  int receive_success = get_all_bytes_data(raw_result); 
  for(int i = 0; i<number_of_mppts_; i++){
    
    for(int j=0; j<4; j++){
       //Results are a voltage for i = 0,2 and current for i = 1,3, change accordingly
      if(j%2) {
        buf[i][j] = get_V_float(raw_result[i][2*j], raw_result[i][2*j+1]);
      } else {
        buf[i][j] = get_A_float(raw_result[i][2*j], raw_result[i][2*j+1]);
      } 
    }
  } 
  return receive_success;
 }


int MPPT::get_int_(uint8_t high_byte, uint8_t low_byte) {
  return 256*(int)high_byte + (int)low_byte;
}


float MPPT::get_A_float(uint8_t high_byte, uint8_t low_byte) {
  int integer_value = get_int_(high_byte, low_byte);
  return (float)integer_value/44.391;
}

float MPPT::get_V_float(uint8_t high_byte, uint8_t low_byte) {
  int integer_value = get_int_(high_byte, low_byte);
  return (float)integer_value/14.2;
}

