/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <cstdint>
#include <thread>
#include <chrono>
#include "MPPT.h"
#include "../../lib-cpp/Debugging/easy_debugging.hpp"
#include "../../include-cpp/can4linux.h"

using namespace std;

using namespace MIO;

using namespace MIO::PowerElectronics;

MPPT::MPPT(CANbus * const canbus, uint8_t number_of_mppts, unsigned long start_address, unsigned long relay_address) : number_of_mppts_(number_of_mppts) {
  if(number_of_mppts_>12) {
    M_ERR<<"WE DO NOT HAVE THAT MANY MPPTS!! (╯°□°）╯︵ ┻━┻";
  }
  
  
  start_address_ = start_address;
  relay_address_ = relay_address;
  
  if(canbus->status()->open){
    canbus_ = canbus;
    if(canbus->status()->status){
      M_OK<<"MPPT SUCCESFULLY INITIALIZED ^_^";
    } else {
      M_WARN<<"THE CANBUS IS NOT RUNNING SO YOU WILL NOT RECEIVE DATA! (ー_ー)!!";
    }
  } else{
    canbus_ = canbus;
    M_ERR<<"MAKE SURE YOU USE A PROPER CANBUS OBJECT WHICH IS OPEN! (ー_ー)!!";
  }
}

int MPPT::get_bytes_from_address(uint8_t buf[], unsigned long address) {
  if(canbus_->status()->open){
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
    M_WARN<<"CELL NUMBER NOT WITHIN RANGE OF MPPTS (╯°□°）╯︵ ┻━┻";
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

int MPPT::set_relay_from_number(bool state, int cell_number) {
  if (set_relay_states_from_number_(cell_number, state)==1) {
    canmsg_t * send_relays = new canmsg_t;
    send_relays->length = 2;
    send_relays->id = relay_address_;
    for(int i = 0; i<send_relays->length; i++){
      send_relays->data[i] = relay_states[i];
    }
    canbus_->write_can(send_relays);
    
    return 1;
  } else {
    return -1;
  }
}

int MPPT::set_relay_from_number(bool state, int cell_number[], int number_of_cells) {
  for(int i = 0; i<number_of_cells; i++){
    if(set_relay_states_from_number_(cell_number[i], state)!=1) {
      return -1;
    }
  }
  
  canmsg_t * send_relays = new canmsg_t;
  send_relays->length = 2;
  send_relays->id = relay_address_;
  for(int i = 0; i<send_relays->length; i++){
    send_relays->data[i] = relay_states[i];
  }
  canbus_->write_can(send_relays);
  return 1;
  
}

int MPPT::set_relay_from_address(bool state, unsigned long address) {
  int cell_number = address - start_address_ + 1;
  return set_relay_from_number(state, address); //This function will do all of the error handling
 }

int MPPT::set_relay_from_address(bool state, unsigned long address[], int number_of_cells) {
  int cell_numbers[number_of_cells];
  
  for(int i = 0; i<number_of_cells; i++){
    cell_numbers[i] = address[i] - start_address_ + 1;
  }
}

int MPPT::set_all_relay(bool state) {
  int cell_numbers[number_of_mppts_];
  for (int i = 0; i<number_of_mppts_; i++){
    cell_numbers[i] = i+1;
  }
  return set_relay_from_number(state, cell_numbers, number_of_mppts_);
 }


int MPPT::get_int_(uint8_t high_byte, uint8_t low_byte) {
  return 256*(int)high_byte + (int)low_byte;
}


float MPPT::get_A_float(uint8_t high_byte, uint8_t low_byte) {
  int integer_value = get_int_(high_byte, low_byte);
  return (float)integer_value/A_factor;
}

float MPPT::get_V_float(uint8_t high_byte, uint8_t low_byte) {
  int integer_value = get_int_(high_byte, low_byte);
  return (float)integer_value/V_factor;
}

int MPPT::set_relay_states_from_number_(int cell_number, bool state){
  if (cell_number>number_of_mppts_|| cell_number<1) {
    M_ERR<<"INVALID CELL NUMBER! NOTHING WAS WRITTEN (='_' )";
    return -1;
  };
  
  if(state){
    if(cell_number>8){
      relay_states[1] |= 1<<(cell_number-9);
    } else {
      relay_states[0] |= 1<<(cell_number-1);
    }
  } else {
    if(cell_number>8){
      relay_states[1] &= ~(1<<(cell_number-9));
    } else {
      relay_states[0] &= ~(1<<(cell_number-1));
    }
  }
  M_DEBUG<<"STATE 1: "<<(int)relay_states[0]<<" | STATE 2: "<<(int)relay_states[1];
  return 1;
  
}

int MPPT::set_relay_from_array(bool state[]) {
  for(int i = 0; i<number_of_mppts_; i++)
    relay_states[i] = state[i];
 
}

int MPPT::set_relay_from_relay_states_() {
  canmsg_t * send_relays = new canmsg_t;
  send_relays->length = 2;
  send_relays->id = relay_address_;
  for(int i = 0; i<send_relays->length; i++){
    send_relays->data[i] = relay_states[i];
  }
  canbus_->write_can(send_relays);
  return 1;

}



MPPT_Controller::MPPT_Controller(MPPT * const m_mppt, structures::PowerInput* const m_power_input, structures::PowerOutput* const m_power_output) : 
    mppt(m_mppt), power_input(m_power_input), power_output(m_power_output){
  
  if(mppt->canbus_->status()){
    M_OK<<"MPPT_CONTROLLER START UP SUCCESSFUL!";
  } else {
    M_ERR<<"TRYING TO START MPPT_CONTROLLER WITHOUT RUNNING CANBUS!";
  }
}

int MPPT_Controller::start(const int delay) {
  if(mppt->canbus_->status()->status){
    if(!reading_state){
      reading_state = true;
      M_OK<<"MPPT_CONTROLLER HAS STARTED READING( ◞･౪･)";
      m_reading_thread_ = thread(&MPPT_Controller::reading_thread_, this, delay);
    } else {
      M_WARN<<"MPPT READING WAS ALREADY STARTED (¬_¬)";
      return -1;
    }
  } else {
    M_ERR<<"CANNOT START READING BECAUSE CANBUS NOT RUNNING (╯•﹏•╰)";
    return -1;
  }
  if(mppt->canbus_->status()->open){
    if(!writing_state){
      writing_state = true;
      M_OK<<"MPPT_CONTROLLER WAS STARTED WRITING ( ◞･౪･)";
      m_reading_thread_ = thread(&MPPT_Controller::writing_thread_, this, delay);
    } else {
      M_WARN<<"MPPT WRITING WAS ALREADY STARTED (¬_¬)";
      return -1;
    }
  } else {
    M_WARN<<"CANNOT START WRITING BECAUSE CANBUS IS NOT OPEN!";
    return -1;
  }
  
  return 1;
}

int MPPT_Controller::stop() {
  if(reading_state){
    reading_state = false;
    m_reading_thread_.join();
    M_OK<<"MPPT STOED READING (ﾉ･ｪ･)ﾉ"; 
  } else {
    M_WARN<<"BMS WRITING HAS NOT YET STARTED (>_<)";
  }
  
  if (writing_state){
    writing_state = false;
    m_writing_thread_.join();
    M_OK<<"MPT STOPPED WRITING (ﾉ･ｪ･)ﾉ";
  } else {
    M_WARN<<"MPPT WRITING HAS NOT YET STARTED (>_<)";
  }
  return 1;
}

int MPPT_Controller::read_data_() {
  float buffer[NUMBER_OF_MPPTS][4];
  int success = mppt->get_all_float_data(buffer);
  for(int i = 0; i<mppt->number_of_mppts_; i++){
    power_input->solar_panels.MPPT_power[i] = buffer[i][0]*buffer[i][1];
    power_input->solar_panels.panel_power[i] = buffer[i][2]*buffer[i][3];
  }
  return success;
}

int MPPT_Controller::write_data_() {
  for(int i = 0; i<mppt->number_of_mppts_; i++){
    return mppt->set_relay_from_array(power_output->solar_panel_states);
  }
}

int MPPT_Controller::reading_thread_(const int delay) {
  M_INFO<<"MPPT STARTS READING WITH DELAY: "<<(long)delay<<"ms ( ͡° ͜ʖ ͡°)";
  while(reading_state){
    read_data_();
    std::this_thread::sleep_for(chrono::milliseconds(delay));
  }
  M_INFO<<"INFO: READING THREAD WAS STOPPED";
}

int MPPT_Controller::writing_thread_(const int delay) {
  M_INFO<<"MPPT STARTS WRITING WITH DELAY: "<<(long)delay<<"ms ( ͡° ͜ʖ ͡°)";
  while(reading_state){
    write_data_();
    std::this_thread::sleep_for(chrono::milliseconds(delay));
  }
  M_INFO<<"INFO: WRITING THREAD WAS STOPPED";
}

