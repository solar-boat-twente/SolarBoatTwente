/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   BMS.cpp
 * Author: Sander Oosterveld
 * 
 * Created on April 11, 2019, 9:27 PM
 */

#include "BMS.h"
#include <iostream>
#include <stdio.h>
#include <thread>
#include <chrono>

#include "../../include/can4linux.h"
#include "../../include/easy_debugging.hpp"

#include "BMS_CANIDs.h"

using namespace MIO;
using namespace MIO::PowerElectronics;
using namespace std;

/*PUBLIC METHODS: */
BMS::BMS(CANbus * const m_canbus):canbus_(m_canbus) {
  CANbus::CanStatus * can_status = canbus_->status();
  if(!can_status->status){
    M_WARN<<"CANBUS NOT RUNNING!";
  } else {
    M_OK<<"BMS SUCCESSFULLY INITIATED";
  }
 }


int BMS::start_reading(structures::PowerInput* power_input){
  //Test if the BMS has already been started, if not starts the reading thread
  if(canbus_->status()->status){
    if(!bms_status->read_state){
      bms_status->read_state = true;
      M_INFO<<"BMS WAS STARTED";
      short int delay = STD_BMS_READ_DELAY;
      m_reading_thread_ = thread(&BMS::reading_thread_, this, power_input, delay);
      return 1;
    } else{
      M_WARN<<"BMS READING WAS ALREADY STARTED";
      return -1;
    }
  } else {
      M_ERR<<"CANNOT START READING BECAUSE CANBUS NOT RUNNING";
      return -1;
  }
}


int BMS::stop_reading(){
  //Tests if the BMS has started, if it has it stops the reading thread
  if(bms_status->read_state){
    bms_status->read_state = false;
    m_reading_thread_.join();
    M_INFO<<"BMS STOPPED READING";
    return 1;
  } else{
    M_WARN<<"BMS READING HAS NOT YET STARTED";
    return -1;
  }
}

int BMS::start_writing(structures::PowerOutput* power_output) {
  //Tests if the BMS has started writing, if not yet started it starts writing.
  if(!bms_status->write_state){
    bms_status->write_state = true;
    M_INFO<<"INFO: BMS STARTED WRITING";
    short int delay = STD_BMS_WRITE_DELAY;
    m_writing_thread_ = thread(&BMS::writing_thread_, this, power_output, delay);
    return 1;
  } else{
    M_WARN<<"BMS WRITING WAS ALREADY STARTED";
    return -1;
  }
}

int BMS::stop_writing() {
  //Tests if the BMS has started, if it has started it stops the writing thread and waits for it to be closed;
  if(bms_status->write_state){
    bms_status->write_state = false;
    m_writing_thread_.join();
    M_INFO<<"BMS STOPPED WRITING";
    return 1;
  } else{
    M_WARN<<"BMS WRITING HAS NOT YET STARTED";
    return -1;
  }
 }


int BMS::write(canmsg_t * const msg) {
  return canbus_->write_can(msg);
}

BMS::BmsStatus* BMS::status() {
  return bms_status;
}



int BMS::get_data_(structures::PowerInput* power_input){
  //The checker to see if all responses were successful
   int success_response1 = 0;
   int success_response2 = 0;
   int success_response_cells = 0;
  
  //Making the storage variables for buffers
  canmsg_t *buffer = new canmsg_t;
  std::vector<uint8_t> *data = new std::vector<uint8_t>;

  //gets data from response 1 and writes to data
  if(canbus_->read_can(CANID_BMS_RESPONSE1, buffer)!=-1){
    //fills the data vector with data from the canmsg_t
    for(int i = 0; i<buffer->length;i++){
      data->push_back(buffer->data[i]);
    }
    //Sends the data to the parser and clears data buffer.
    M_OK<<"READING RESPONSE 1 SUCCESSFUL!";
    success_response1 = parse_response1_(data, power_input);
    data->clear();
  }else{
    M_WARN<<"WARNING: TRIED TO READ NON-EXISTING DATA FROM "<<CANID_BMS_RESPONSE1;
  }
  
  //get data from response 2
  if(canbus_->read_can(CANID_BMS_RESPONSE2, buffer)!=-1){
    std::cout<<"Buffer length: "<<buffer->length<<endl;
    //short int length = buffer->length;
    for(int i = 0; i<buffer->length;i++){
      data->push_back(buffer->data[i]);
    }

    //Sending the data to the parser.
    M_OK<<"READING RESPONSE 2 SUCCESSFUL!";
    success_response2 = parse_response2_(data, power_input);
    data->clear();
  } else{
    M_WARN<<"WARNING: TRIED TO READ NON-EXISTING DATA FROM "<<CANID_BMS_RESPONSE2;
  }
  
  //get data for the cell voltages
  short int cells_remaining = MAX_CELLS;
  short int counter = 0;
  while(cells_remaining>0) {
    if(canbus_->read_can(CANID_BMS_CELL_VOLTAGE_START+counter, buffer)!=-1) {
      //short int length = buffer->length;
      for(int i=0; i<buffer->length;i++) {
        data->push_back(buffer->data[i]);
      } 
    } else {
      M_WARN<<"WARNING: TRIED TO READ NON-EXISTING DATA FROM "<<CANID_BMS_CELL_VOLTAGE_START+counter;
    }
    cells_remaining-=4;
    counter++;
  }
  //Tests if all the required data as been read from all of the cells
  if(data->size()>=MAX_CELLS) {
    M_OK<<"READING CELL VOLTAGES SUCCESSFUL!";
    success_response_cells = parse_cell_voltages(data, power_input);
  }
  
  //Frees up the space
  delete buffer;
  delete data;
  
  //If all the responses were successful returns success
  if(success_response1 & success_response2 & success_response_cells == 1){
    return 1;
  } else{
    return 0;
  }
}

int BMS::parse_response1_(std::vector<uint8_t> * bytes, structures::PowerInput* power_input) {
  if(bytes->size()==8) {
    power_input->battery.state_of_charge = 0.5*(*bytes)[0];
    power_input->battery.total_voltage = (256*(*bytes)[1]+(*bytes)[2])*0.002;
    power_input->battery.total_current = (256*(*bytes)[3]+ (*bytes)[4])*0.020;
    power_input->battery.error_number = (*bytes)[6];
    power_input->battery.error_location = (*bytes)[7];
    M_OK<<"PARSING RESPONSE 1 SUCCESSFUL!";
    M_DEBUG<<"RESULT FROM RESPONSE 1 IS: \n"<<"soc: "<<power_input->battery.state_of_charge<< "tot_V: "<<power_input->battery.total_voltage<<
        "tot_I: "<<power_input->battery.total_current<< "er_num: "<<power_input->battery.error_number<< "er_loc: "<<power_input->battery.error_location;
    return 1;
  } else {
    M_ERR<<"INCORRECT MESSAGE SIZE FOR ID = "<<CANID_BMS_RESPONSE1<<" Size was equal to "<<bytes->size();
    return -1;
  }
}

int BMS::parse_response2_(std::vector<uint8_t> *bytes, structures::PowerInput* power_input) {
  if(bytes->size()==5) {
    power_input->battery.max_temp = (*bytes)[0]-127;
    power_input->battery.min_temp = (*bytes)[1]-127;

    if((*bytes)[2]==1) {
      power_input->battery.balance_state = true;
    } else{
      power_input->battery.balance_state = false;
    }
    

    if((*bytes)[3]==1) {
      power_input->battery.contactor_ready = true;
    } else {
      power_input->battery.contactor_ready = false;
    }
    
    //Read the contactor status from the 5th byte;
    if((*bytes)[4]==1) {
      power_input->battery.contactor_status = true;
    } else {
      power_input->battery.contactor_status = false;
    }
    M_OK<<"PARSING RESPONSE 2 SUCCESSFULL";
    M_DEBUG<<"RESULT FROM RESPONSE 2 IS: \n"<<"max_temp: "<<power_input->battery.max_temp<< "min_temp: "<<power_input->battery.min_temp<<
        "bal_state: "<<power_input->battery.balance_state<< "cont_read: "<<power_input->battery.contactor_ready<< "cont_state: "<<power_input->battery.contactor_status;
  } else {
    M_ERR<<"INCORRECT MESSAGE SIZE FOR ID = "<<CANID_BMS_RESPONSE2<<" Size was equal to "<<bytes->size();
  }
  
}

int BMS::parse_cell_voltages(std::vector<uint8_t> *bytes, structures::PowerInput* power_input){
  
  for(int i = 0; i<MAX_CELLS; i++) {
    power_input->battery.cel_voltages[i]=((*bytes)[2*i]*256+(*bytes)[2*i+1])*0.001;
  }
  
  M_DEBUG<<"RESULT FROM CELL VOLTAGES IS: ";
  for(int i = 0; i<12; i++){
    cout<<"Cell "<<i<<": "<<power_input->battery.cel_voltages[i];
    if(i%4==3){
      cout<<endl;
    }
  }    
  
}

void BMS::reading_thread_(structures::PowerInput * const power_input, short int delay) {
  M_INFO<<"READING THREAD HAS STARTED WITH DELAY: "<<delay;
  while(bms_status->read_state){
    get_data_(power_input);
    std::this_thread::sleep_for(chrono::milliseconds(delay));
  }
  M_INFO<<"READING THREAD WAS STOPPED";
}

int BMS::write_data_(structures::PowerOutput* power_output) {
  //Write whether the contactor should be one:
  M_DEBUG<<"WRITING BMS TX MESSAGE";
  canmsg_t * bms_tx = new canmsg_t;
  bms_tx->id = CANID_BMS_TX;
  bms_tx->length = 2;
  bms_tx->data[0] = power_output->contractor_control;
  bms_tx->data[1] = power_output->balancing_control;
  write(bms_tx);
}



void BMS::writing_thread_(structures::PowerOutput* power_output, short int delay) {
  M_INFO<<"WRITING THREAD WAS STARTED WITH DELAY: "<<delay;
  while(bms_status->write_state){
    write_data_(power_output);
    std::this_thread::sleep_for(chrono::milliseconds(delay));
  }
  M_INFO<<"INFO: WRITING THREAD WAS STOPPED";
 }
