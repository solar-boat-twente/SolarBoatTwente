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
#include "src-cpp/Genasun_Watt_Sensor/MPPT.h"
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
    
  //Start the BMS
  BMS * m_bms = new BMS(canbus_bms);
  m_bms->start_reading(power_input);
  
  MPPT_Box * mppt_box = new MPPT_Box(canbus_bms);
  
  //construct message for bms
  canmsg_t * bms_tx = new canmsg_t;
  bms_tx->id = CANID_BMS_TX;
  bms_tx->length = 2;
  bms_tx->data[0] = 0x01;
  bms_tx->data[1] = 0x0;
  
  //Construct message for driver
  canmsg_t * driver_tx = new canmsg_t;
  driver_tx->id = 0xcf;
  driver_tx->length = 4;
  driver_tx->data[0] = 0;
  driver_tx->data[1] = 2;
  driver_tx->data[2] = 0x19;
  driver_tx->data[3] = 0x99;
  
  //Construct sampling message;
  canmsg_t driver_sample;
  driver_sample.id = 0xC7;
  driver_sample.length = 8;
  
  driver_sample.data[0] = 0x0;
  driver_sample.data[1] = 0x08;
  driver_sample.data[2] = 0;
  driver_sample.data[3] = 0xff;
  driver_sample.data[4] = 0x03;
  driver_sample.data[5] = 0xE8;
  driver_sample.data[6] = 0x0;
  driver_sample.data[7] = 0x64;
  
  //First turn off all the mppts before turning them back on..
  mppt_box->set_all_relay(false);
  for(int i = 1; i<11; i++){
    mppt_box->set_relay_from_number(true, 11-i);
    this_thread::sleep_for(chrono::milliseconds(100));
  } 
  
  uint8_t required_speed_percent = 50;
  short signed int real_speed = 0;
  
  short int CORRECTION_FACTOR = 100; //value between 0 and 1, set correct for max current.
  
  int counter = 0;
  
  ofstream file;
  file.open("/root/logfiles/log230519.log", std::ios::app);
  //buffer for mppt data
  float mppt_buffer[10][4];

  //buffers for driver data;
  canmsg_t  supply_1_data;
  canmsg_t  motor_1_data;
  canmsg_t  motor_2_data;
  canmsg_t  driver_state_data;
  canmsg_t  reference_data;
  
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
    
    if (counter%5 == 1){
      mppt_box->get_all_float_data(mppt_buffer);
      
      canbus_driver->read_can(0xd0, &driver_state_data);
      canbus_driver->read_can(0xd8, &reference_data);
      canbus_driver->read_can(0xe0, &supply_1_data);
      canbus_driver->read_can(0xf0, &motor_1_data);
      canbus_driver->read_can(0xf8, &motor_2_data);      
      
      
      file<<counter<<power_input->battery.max_temp << ","<<power_input->battery.total_current << ","<<power_input->battery.total_voltage << ","
          <<power_input->battery.state_of_charge << ","<<user_input->steer.raw_throttle;
      file<<",Solar,";
      
      for(int i = 0; i<4; i++){
        for (int j = 0; j<10; j++){
          file<<mppt_buffer[j][i]<<",";
        }
      file<<" ,";
      }
      file<<"Driver,";
      for(int i = 0; i<8; i++){
        file<<(int)driver_state_data.data[i]<<",";
      }
      " ,";
      for(int i = 0; i<8; i++){
        file<<(int)reference_data.data[i]<<",";
      }
      " ,";
      for(int i = 0; i<8; i++){
        file<<(int)supply_1_data.data[i]<<",";
      }
      " ,";
      for(int i = 0; i<8; i++){
        file<<(int)motor_1_data.data[i]<<",";
      }
      " ,";      
      for(int i = 0; i<8; i++){
        file<<(int)motor_2_data.data[i]<<",";
      }      
      file<<"\n"<<flush;
    } 
    
    if (counter%10 == 1){
      
      
    }
    
  }
  
  
  return 0;
}