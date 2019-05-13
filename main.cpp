/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: Sander Oosterveld
 *
 * Created on April 30, 2019, 9:37 AM
 */
#include <iostream>
#include <cstdlib>
#include <stdlib.h>
#include <string>
#include <fcntl.h>
#include <unistd.h> 
#include "solarboattwente.h"
//#include "lib-cpp/Logging/thread.h"
//#include "lib-cpp/Logging/logging.h"
#include "src-cpp/Battery_Magagement_System/BMS.h"
#include "src-cpp/Battery_Magagement_System/BMS_CANIDs.h"
#include "src-cpp/Control_Wheel/Control_Wheel.hpp"
//#include "lib-cpp/Logging/easylogging++.h"



using namespace std;
using namespace MIO;
using namespace PowerElectronics;
using namespace structures;


int main(int argc, const char** argv) {
  close(10);
  close(11);
  Serial * serial_wheel = new Serial("/dev/ttyUSB0");
  PowerInput * power_input = new PowerInput;
  PowerOutput * power_output = new PowerOutput;
  UserInput * user_input = new UserInput;
  user_input->steer.raw_throttle = 0;
  //std::cout<<open("/dev/can0", O_RDWR);
  CANbus * canbus_bms = new CANbus("/dev/can0", 1);
  CANbus * canbus_driver = new CANbus("/dev/can1", 1);
  canbus_driver->close_can();
  canbus_driver->open_can(O_RDWR|O_NONBLOCK);
  UI::ControlWheel * control_wheel = new UI::ControlWheel(serial_wheel);
  control_wheel->start_reading(user_input, 50);
  
  
  
  //canbus_driver->start(100);
  //canbus_bms->start(100);
  //BMS * m_bms = new BMS(canbus_bms);
  //m_bms->start_reading(power_input);
  
  canmsg_t * bms_tx = new canmsg_t;
  bms_tx->id = CANID_BMS_TX;
  bms_tx->length = 2;
  bms_tx->data[0] = 0x01;
  bms_tx->data[1] = 0x0;
  
  canmsg_t * driver_tx = new canmsg_t;
  driver_tx->id = 0xcf;
  driver_tx->length = 4;
  driver_tx->data[0] = 0;
  driver_tx->data[1] = 2;
  driver_tx->data[2] = 0x19;
  driver_tx->data[3] = 0x99;
  
  uint8_t required_speed_percent = 50;
  short signed int real_speed = 0;
  
  short int CORRECTION_FACTOR = 100; //value between 0 and 1, set correct for max current.
  
  
  while (true){
    real_speed = 32 * user_input->steer.raw_throttle * CORRECTION_FACTOR/100;
    std::cout<<real_speed<<std::endl;
    if (user_input->steer.reverse){
      real_speed = -real_speed;
    }
    driver_tx->data[2] = real_speed>>8;
    driver_tx->data[3] = real_speed&0xFF;
    
    
    
    canbus_bms->write_can(bms_tx);
    this_thread::sleep_for(chrono::milliseconds(100));
    canbus_driver->write_can(driver_tx);
    
    
  }
  
  
  return 0;
}