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
#include <thread>
#include "lib-cpp/Logging/thread.h"
#include "lib-cpp/Logging/logging.h"
#include "src-cpp/Battery_Magagement_System/BMS.h"
#include "src-cpp/Battery_Magagement_System/BMS_CANIDs.h"
#include "src-cpp/Control_Wheel/Control_Wheel.hpp"


using namespace std;
using namespace MIO;
using namespace UI;
using namespace structures;


void update_MC(UserInput * user_input){
  canmsg_t driver_tx;
  driver_tx.id = 
  
}


int main(int argc, const char** argv) {
  
  CANbus * canbus_pwr = new CANbus("/dev/can0", 1);
  canbus_pwr->start(100);
  
  
  structures::PowerInput * power_input = new PowerInput;
  structures::PowerOutput * power_output = new PowerOutput;
  structures::UserInput * user_input = new UserInput;
  
  PowerElectronics::BMS * bms = new PowerElectronics::BMS(canbus_pwr);
  bms->start_reading(power_input);
  
  Thread threads;
  
  threads.CreateThreads();
  
  Logger user_power_logger("/root/SolarBoat2019/SolarBoatTwente/config/user_power.conf");
  while (true){
    user_power_logger.write_struct_user_power(power_input, power_output, user_input);
    this_thread::sleep_for(chrono::milliseconds(200));
  }
  
  
  return 0;
}