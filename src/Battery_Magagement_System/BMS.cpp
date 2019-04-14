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
#include "../Colors.h"
#include "BMS_CANIDs.h"

using namespace MIO;
using namespace MIO::PowerElectronics;
using namespace std;

BMS::BMS(CANbus * const m_canbus):canbus_(m_canbus) {
  CanStatus * can_status = canbus_->status();
  if(!can_status->status){
    fprintf(stderr, WARN("WARNING: CANBUS NOT RUNNING!\n\n"));
  }
 }


int BMS::start_reading(structures::PowerInput* power_input){
  if(!bms_status->read_state){
    bms_status->read_state = true;
    cout<<INFO("INFO: BMS WAS STARTED")<<endl;
    short int delay = STD_BMS_READ_DELAY;
    m_reading_thread_ = thread(&BMS::reading_thread_, this, power_input, delay);
  } else{
    cout<<WARN("WARNING: BMS READING WAS ALREADY STARTED")<<endl;
  }
}


int BMS::stop_reading(){
  if(bms_status){
    bms_status->read_state = false;
    m_reading_thread_.join();
  } else{
    cout<<WARN("WARNING: BMS READING HAS NOT YET STARTED")<<endl;
  }
}

int BMS::start_writing(structures::PowerOutput* power_output) {
  if(!bms_status->write_state){
    bms_status->write_state = true;
    cout<<INFO("INFO: BMS WAS STARTED")<<endl;
    short int delay = STD_BMS_WRITE_DELAY;
    m_writing_thread_ = thread(&BMS::writing_thread_, this, power_output, delay);
  } else{
    cout<<WARN("WARNING: BMS WRITING WAS ALREADY STARTED")<<endl;
  }
}

int BMS::stop_writing() {
  if(bms_status->write_state){
    bms_status->write_state = false;
    m_writing_thread_.join();
  } else{
    cout<<WARN("WARNING: BMS WRITING HAS NOT YET STARTED")<<endl;
  }
 }


int BMS::write(canmsg_t * const msg) {
  return canbus_->write(msg);
}

BmsStatus* BMS::status() {
  return bms_status;
}



int BMS::get_data_(structures::PowerInput* power_input){
   int success_response1 = 0;
   int success_response2 = 0;
   int success_response_cells = 0;
   
  canmsg_t *buffer = new canmsg_t;
  std::vector<uint8_t> *data = new std::vector<uint8_t>;

  //get data from response 1
  if(canbus_->read(CANID_BMS_RESPONSE1, buffer)!=-1){
    //short int length = buffer->length;
    cout<<buffer->length<<endl;
    for(int i = 0; i<buffer->length;i++){
      data->push_back(buffer->data[i]);
    }
    //Sending the data to the parser.
     success_response1 = parse_response1_(data, power_input);
     data->clear();
  }else{
    fprintf(stderr, WARN("WARNING: TRIED TO READ NON-EXISTING DATA FROM 0x%x\n"),CANID_BMS_RESPONSE1);
  }
  
  //get data from response 2
  if(canbus_->read(CANID_BMS_RESPONSE2, buffer)!=-1){
    std::cout<<"Buffer length: "<<buffer->length<<endl;
    //short int length = buffer->length;
    for(int i = 0; i<buffer->length;i++){
      data->push_back(buffer->data[i]);
    }
    printf("Going to parsing: \n");

    //Sending the data to the parser.
     int sucess_response2 = parse_response2_(data, power_input);
     data->clear();
  } else{
    fprintf(stderr, WARN("WARNING: TRIED TO READ NON-EXISTING DATA FROM 0x%x\n"),CANID_BMS_RESPONSE2);
  }
  
  //get data for the cell voltages
  short int cells_remaining = MAX_CELLS;
  short int counter = 0;
  while(cells_remaining>0){
    if(canbus_->read(CANID_BMS_CELL_VOLTAGE_START+counter, buffer)!=-1){
      //short int length = buffer->length;
      for(int i=0; i<buffer->length;i++){
        data->push_back(buffer->data[i]);
      } 
    } else {
      fprintf(stderr, WARN("WARNING: TRIED TO READ NON-EXISTING DATA FROM 0x%x\n"),CANID_BMS_CELL_VOLTAGE_START+counter);
      }
    cells_remaining-=4;
    counter++;
  }
  
  if(data->size()>=MAX_CELLS) {
    success_response_cells = parse_cell_voltages(data, power_input);
  }
  
  //Freeing up the space
  delete buffer;
  delete data;
  
  if(success_response1 & success_response2 & success_response_cells == 1){
    return 1;
  } else{
    return 0;
  }
}

int BMS::parse_response1_(std::vector<uint8_t> * bytes, structures::PowerInput* power_input) {
  printf(INFO("INFO: STARTING PARSING RESPONSE 1\n"));
  if(bytes->size()==8) {
    power_input->battery.state_of_charge = 0.5*(*bytes)[0];
    power_input->battery.total_voltage = (256*(*bytes)[1]+(*bytes)[2])*0.002;
    power_input->battery.total_current = (256*(*bytes)[3]+ (*bytes)[4])*0.020;
    power_input->battery.error_number = (*bytes)[6];
    power_input->battery.error_location = (*bytes)[7];
    return 1;
  } else {
    fprintf(stderr, ERR("ERROR: INCORRECT MESSAGE SIZE FOR ID = 0x%x\nSize was equal to %i\n"), CANID_BMS_RESPONSE1, bytes->size());
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
  } else {
    fprintf(stderr, ERR("ERROR: INCORRECT MESSAGE SIZE FOR ID = 0x%x\nSize was equal to: %i\n"), CANID_BMS_RESPONSE2, bytes->size());
  }
  
}

int BMS::parse_cell_voltages(std::vector<uint8_t> *bytes, structures::PowerInput* power_input){
  
  for(int i = 0; i<MAX_CELLS; i++) {
    power_input->battery.cel_voltages[i]=((*bytes)[2*i]*256+(*bytes)[2*i+1])*0.001;
  }
  
}

void BMS::reading_thread_(structures::PowerInput * const power_input, short int delay) {
  cout<<INFO("INFO: READING THREAD WAS STARTED")<<endl;
  while(bms_status->read_state){
    get_data_(power_input);
    std::this_thread::sleep_for(chrono::milliseconds(delay));
  }
  cout<<INFO("INFO: READING THREAD WAS STOPPED")<<endl;
}

int BMS::write_data_(structures::PowerOutput* power_output) {
  //Write whether the contactor should be one:
  canmsg_t * bms_tx = new canmsg_t;
  bms_tx->id = CANID_BMS_TX;
  bms_tx->length = 2;
  bms_tx->data[0] = power_output->contractor_control;
  bms_tx->data[1] = power_output->balancing_control;
  write(bms_tx);
}



void BMS::writing_thread_(structures::PowerOutput* power_output, short int delay) {
  cout<<INFO("INFO: WRITING THREAD WAS STARTED")<<endl;
  while(bms_status->write_state){
    write_data_(power_output);
    std::this_thread::sleep_for(chrono::milliseconds(delay));
  }
  cout<<INFO("INFO: WRITING THREAD WAS STOPPED")<<endl;
 }
