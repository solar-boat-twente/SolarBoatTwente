///*
// * To change this license header, choose License Headers in Project Properties.
// * To change this template file, choose Tools | Templates
// * and open the template in the editor.
// */
//
///* 
// * File:   main.cpp
// * Author: Sander Oosterveld
// *
// * Created on April 30, 2019, 9:37 AM
// */
//#include <iostream>
//#include <cstdlib>
//#include <stdlib.h>
//#include <string>
//#include <fcntl.h>
//#include <unistd.h> 
//#include "solarboattwente.h"
//#include <thread>
//#include "lib-cpp/Logging/thread.h"
//#include "lib-cpp/Logging/logging.h"
//#include "src-cpp/Battery_Magagement_System/BMS.h"
//#include "src-cpp/Battery_Magagement_System/BMS_CANIDs.h"
//#include "src-cpp/Control_Wheel/Control_Wheel.hpp"
//
//
//using namespace std;
//using namespace MIO;
//using namespace UI;
//using namespace structures;
//
//
//void update_MC(UserInput * user_input){
//  canmsg_t driver_tx;
//  driver_tx.id = 
//  
//}
//
//
//int main(int argc, const char** argv) {
//  
//  CANbus * canbus_pwr = new CANbus("/dev/can0", 1);
//  canbus_pwr->start(100);
//  
//  
//  structures::PowerInput * power_input = new PowerInput;
//  structures::PowerOutput * power_output = new PowerOutput;
//  structures::UserInput * user_input = new UserInput;
//  
//  PowerElectronics::BMS * bms = new PowerElectronics::BMS(canbus_pwr);
//  bms->start_reading(power_input);
//  
//  Thread threads;
//  
//  threads.CreateThreads();
//  
//  Logger user_power_logger("/root/SolarBoat2019/SolarBoatTwente/config/user_power.conf");
//  while (true){
//    user_power_logger.write_struct_user_power(power_input, power_output, user_input);
//    this_thread::sleep_for(chrono::milliseconds(200));
//  }
//  
//  
//  return 0;
//}

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
#include <fstream>
#include <sstream>

#include "solarboattwente.h"
//#include "lib-cpp/Logging/thread.h"
//#include "lib-cpp/Logging/logging.h"
#include "src-cpp/Battery_Magagement_System/BMS.h"
#include "src-cpp/Battery_Magagement_System/BMS_CANIDs.h"
#include "src-cpp/Control_Wheel/Control_Wheel.hpp"
#include "lib-cpp/Logging/thread.h"
//#include "lib-cpp/Logging/easylogging++.h"



using namespace std;
using namespace MIO;
using namespace PowerElectronics;
using namespace structures;


int main(int argc, const char** argv) {
  //Open up the data acquisition parts
  Serial * serial_wheel = new Serial("/dev/steer");
  CANbus * canbus_bms = new CANbus("/dev/can0", 1);
  CANbus * canbus_driver = new CANbus("/dev/can1", 1);
  
  
  //Open up the global structures
  PowerInput * power_input = new PowerInput;
  PowerOutput * power_output = new PowerOutput;
  UserInput * user_input = new UserInput;
  ControlData * control_data = new ControlData;
  
  //Starting with reading from the CANbus
  canbus_bms->start(100);
  canbus_driver->start(100);
  
  //Start the Steering Wheel
  UI::ControlWheel * control_wheel = new UI::ControlWheel(serial_wheel);
  control_wheel->start_reading(user_input, 50);
  
  //Start up the screen
  Thread threads;
  threads.CreateThreads();
  
  
  //canbus_driver->start(100);
  BMS * m_bms = new BMS(canbus_bms);
  m_bms->start_reading(power_input);
  
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
  
  int counter = 0;
  
  ofstream file;
  file.open("/root/logfiles/log210519.log", std::ios::app);
  
  while (true){
    counter++;
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
    
    //threads.writeUserPower(power_input, power_output, user_input, 0, 1);
    //threads.writeControlData(control_data, 0 ,1);
    std::cout<<"WRITTEN TO PIPES";
    if (counter%5 == 1){
      file<<counter<<power_input->battery.max_temp << ","<<power_input->battery.total_current << ","<<power_input->battery.total_voltage << ","
          <<power_input->battery.state_of_charge << ","<<user_input->steer.raw_throttle<<"\n";
      file<<flush;
    }  
    
  }
  
  
  return 0;
}