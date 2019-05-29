#include <iostream>
#include <cstdlib>
#include <stdlib.h>
#include <string>
#include <fcntl.h>
#include <unistd.h> 
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <cstdlib>
#include <math.h>
//#include <chrono>
#include <ctime>
#include <ratio>

#include "solarboattwente.h"
//#include "lib-cpp/Logging/thread.h"
//#include "lib-cpp/Logging/logging.h"
#include "src-cpp/Battery_Magagement_System/BMS.h"
#include "src-cpp/Battery_Magagement_System/BMS_CANIDs.h"
#include "src-cpp/Control_Wheel/Control_Wheel.hpp"
#include "lib-cpp/Logging/thread.h"
#include "src-cpp/Genasun_Watt_Sensor/MPPT.h"
//#include "lib-cpp/Logging/easylogging++.h"
#include "src-cpp/Control_System/DataStore.h"
#include "src-cpp/Control_System/Filtered_data.h"
#include "src-cpp/Control_System/Sensor.h"
#include "src-cpp/Control_System/ComplementaryFilter.h"
#include "src-cpp/Control_System/PID_caller.h"
#include "src-cpp/Control_System/Force_to_wing_angle.h"
#include "src-cpp/Control_System/Daan_Test1_maxon.h"
#include "lib-cpp/Canbus/canbus.h"
#include "lib-cpp/Debugging/easy_debugging.hpp"
#include "src-cpp/Motor/Motor.hpp"
#include "src-cpp/Button_Box/Button_Box.hpp"


using namespace std;
using namespace MIO;
using namespace PowerElectronics;
using namespace structures;

//Define everything 
//Open up the data acquisition parts  
Serial * serial_wheel = new Serial("/dev/steer");
CANbus * canbus_bms = new CANbus("/dev/can1", 1);
CANbus * canbus_driver = new CANbus("/dev/can0", 1);
UI::ADAM

//Open up the global structures
PowerInput * power_input = new PowerInput;
PowerOutput * power_output = new PowerOutput;
UserInput * user_input = new UserInput;
ControlData * control_data = new ControlData;

UI::ControlWheel * control_wheel = new UI::ControlWheel(serial_wheel);
BMS * m_bms = new BMS(canbus_bms);
canmsg_t * bms_tx = new canmsg_t;
canmsg_t * driver_tx = new canmsg_t;
MPPT_Box * mppt_box = new MPPT_Box(canbus_driver);

//Define everything from the control system
DataStore * xsens_data = new DataStore();
DataStore * ruwe_data = new DataStore();
DataStore * filtered_data = new DataStore();
DataStore * complementary_data= new DataStore();
DataStore * pid_data = new DataStore();
DataStore * FtoW_data = new DataStore();
RuwDataFilter * filter = new RuwDataFilter();
ComplementaryFilter * com_filter = new ComplementaryFilter();
PID_caller * PIDAAN = new PID_caller();
control::ForceToWingAngle * FtoW = new control::ForceToWingAngle();
//CANbus * canbus1 = new CANbus("can1", 1);

EPOS * maxon1 = new EPOS(canbus_bms,1);
EPOS * maxon2 = new EPOS(canbus_bms,2);
EPOS * maxon4 = new EPOS(canbus_bms,4);

void floating(){
  //Starting with reading from the CANbus
  canbus_bms->start(5);
  canbus_driver->start(5);
  
  //Start the Steering Wheel
  control_wheel->start_reading(user_input, 50);
    
  //Start the BMS
  m_bms->start_reading(power_input);
}

void controlsystem(){ 
//int motor_on = 0;
//  canmsg_t test_buffer;
//  while(true){
//      motor_on = canbus_driver->read_can(0xd0, &test_buffer);
//      if (motor_on == 1){
//        break;
//      }
//      this_thread::sleep_for(chrono::milliseconds(300));
//    }
//  
typedef std::chrono::microseconds ms;
typedef std::chrono::milliseconds mls;
  /* -----------------------------------------------------------------------------
All three motors are going to home.
----------------------------------------------------------------------------- */    
this_thread::sleep_for(chrono::milliseconds(2000));
maxon1->Homing();
this_thread::sleep_for(chrono::milliseconds(2000));
maxon2->Homing();
this_thread::sleep_for(chrono::milliseconds(2000));
//maxon4->Homing();
this_thread::sleep_for(chrono::milliseconds(500));

maxon1->HomingCheck();
maxon2->HomingCheck();
//maxon4->HomingCheck();

/* -----------------------------------------------------------------------------
All three motors are going in the startpositionmode.
----------------------------------------------------------------------------- */    
maxon1->StartPositionMode();
//this_thread::sleep_for(chrono::milliseconds(1000));      
maxon2->StartPositionMode();
//this_thread::sleep_for(chrono::milliseconds(1000));
//maxon4->StartPositionMode();
//this_thread::sleep_for(chrono::milliseconds(10000));


/*NEW VERSION FOR HOMING MAXON --> A LOT FASTER!*/
//maxon1->start_homing();
//maxon2->start_homing();
//maxon4->start_homing();
//
//maxon1->check_homing_block();
//maxon2->check_homing_block();
//maxon4->check_homing_block();
//
//maxon1->start_position_mode();
//maxon2->start_position_mode();
//maxon4->start_position_mode();
//
//maxon1->check_position_mode();
//maxon2->check_position_mode();
//maxon4->check_position_mode();

std::this_thread::sleep_for(std::chrono::milliseconds(500));  // wait 500 milliseconds, because otherwise the xsens can enter the configuration mode
  
filter->m_ruwe_state_data = ruwe_data;     // (2)
filter->m_filtered_data = filtered_data;        // (3)
com_filter->m_filtered_data = filtered_data; //(4)
com_filter->m_complementary_data = complementary_data; //(5)
PIDAAN->m_complementary_data = complementary_data;  //(6)
PIDAAN->m_PID_data = pid_data; //(7)
FtoW->m_complementary_data = complementary_data; //(8)
FtoW->m_PID_data = pid_data; //(9)
FtoW->m_FtoW_data = FtoW_data; //(10)
FtoW->m_xsens_state_data= xsens_data;
maxon1->m_FtoW_data = FtoW_data; //(11)
maxon2->m_FtoW_data = FtoW_data; //(12)
maxon4->m_FtoW_data = FtoW_data; //(13)
    
Serial * m_serial = new Serial("/dev/xsense", 9600);
Xsens * m_xsens = new Xsens(); //initialise class Xsens
Sensor * de_sensor = new Sensor(m_xsens,m_serial);

m_xsens->m_xsens_state_data = xsens_data; //(0)
de_sensor->m_xsens_state_data = xsens_data;
de_sensor->m_ruwe_state_data = ruwe_data;  // (1)
int counter_control_front = 4;        //80Hz waar de loop op loopt
int counter_control_back = 8;
  while (true) {
    std::chrono::high_resolution_clock::time_point t1= std::chrono::high_resolution_clock::now();
    de_sensor->get_data();
    if (counter_control_front == 0){     //Voorvleugels lopen op 20Hz
      filter->FilterIt();
      com_filter->CalculateRealHeight();            
      PIDAAN->PID_in();
      FtoW->MMA();
      
      maxon1->Move();
      maxon2->Move();
      //this_thread::sleep_for(chrono::milliseconds(5));
      counter_control_front = 5;
    }
    if (counter_control_back == 0){ //achtervleugel op 10 Hz
      filter->FilterIt();
      com_filter->CalculateRealHeight();            
      PIDAAN->PID_in();
      FtoW->MMA();  
      //maxon4->Move();
      counter_control_back=9;
    }
    counter_control_front--;
    counter_control_back--;
    std::chrono::high_resolution_clock::time_point t2= std::chrono::high_resolution_clock::now();
    ms d = std::chrono::duration_cast<ms>(t2-t1);
    int t_total = 12500-d.count();
    printf("loop tijd is %i",t_total);
    this_thread::sleep_for(chrono::microseconds(t_total));
  }
}

int main(int argc, const char** argv) {  
  
  
  Motor * motor  = new Motor(canbus_driver, 15);
  
  std::thread thread_floating(floating);
  //std::thread thread_controlsystem(controlsystem);
  
  //construct message for bms
  bms_tx->id = CANID_BMS_TX;
  bms_tx->length = 2;
  bms_tx->data[0] = 0x01;
  bms_tx->data[1] = 0x0;
  
  //Construct message for driver
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
  
  canmsg_t mppt_tx;
  mppt_tx.id = 0x3c;
  mppt_tx.length = 2;
  mppt_tx.data[0] = 0xff;
  mppt_tx.data[1] = 0xff;
  
  //First turn off all the mppts before turning them back on..
//  mppt_box->set_all_relay(false);
//  for(int i = 1; i<11; i++){
//    mppt_box->set_relay_from_number(true, 11-i);
//    this_thread::sleep_for(chrono::milliseconds(100));
//  } 
  
  uint8_t required_speed_percent = 50;
  short signed int real_speed = 0;
  
  short int CORRECTION_FACTOR = 100; //value between 0 and 1, set correct for max current.
  
  int counter = 0;
  
  ofstream file;
  file.open("/root/logfiles/log290519.log", std::ios::app);
  //buffer for mppt data
  ofstream control_file;
  control_file.open("/root/logfiles/control_240519.txt", std::ios::app);
  
  ofstream driver_file;
  driver_file.open("/root/logfiles/driver_2019_05_29_11_33", std::ios::app);
  
  float mppt_buffer[10][4];

  //buffers for driver data;
  canmsg_t  supply_1_data;
  canmsg_t  motor_1_data;
  canmsg_t  motor_2_data;
  canmsg_t  driver_state_data;
  canmsg_t  reference_data;
  
  int no_driver_data_counter = 0;
  
  while (true){
    counter++;
    real_speed = 32 * user_input->steer.raw_throttle * CORRECTION_FACTOR/100;
    if (user_input->steer.reverse){
      real_speed = -real_speed;
    }
    driver_tx->data[2] = real_speed>>8;
    driver_tx->data[3] = real_speed&0xFF;

    canbus_bms->write_can(bms_tx);
    this_thread::sleep_for(chrono::milliseconds(100));
    canbus_driver->write_can(driver_tx);
    
    if (counter%5 == 1){
    for(int i = 1; i<11; i++){
      //mppt_box->set_relay_from_number(true, 11-i);
      //this_thread::sleep_for(chrono::milliseconds(5));
    } 
      if(motor->update_data_from_driver()){
        no_driver_data_counter = 0;
      } else {
        no_driver_data_counter++;
      }
      if(no_driver_data_counter>5){
        motor->setup_sampling();
        no_driver_data_counter = 0;
      }
      
      driver_file<<no_driver_data_counter<<","<<counter<<","<<(int)motor->values.driver_temp<<","<<motor->values.link_voltage<<","<<motor->values.phase_current<<","<<motor->values.motor_power<<","<<motor->values.rotor_speed
          <<","<<motor->values.supply_current<<","<<motor->values.supply_voltage<<","<<motor->values.torque<<","<<(int)motor->values.motor_mode<<","<<(int)real_speed<<"\n"<<flush;
      
      //screen_threads.writeUserPower(power_input, power_output, user_input, 0, 1);
    }
    
    if (counter%10 == 1){
      //canbus_driver->write_can(&mppt_tx);
      mppt_box->get_all_float_data(mppt_buffer);
//      
//      canbus_driver->read_can(0xd0, &driver_state_data);
//      canbus_driver->read_can(0xd8, &reference_data);
//      canbus_driver->read_can(0xe0, &supply_1_data);
//      canbus_driver->read_can(0xf0, &motor_1_data);
//      canbus_driver->read_can(0xf8, &motor_2_data);      
//      
      
      file<<counter<<","<<power_input->battery.max_temp << ","<<power_input->battery.total_current << ","<<power_input->battery.total_voltage << ","
          <<power_input->battery.state_of_charge << ","<<user_input->steer.raw_throttle;
      file<<",Solar,";
      
      for(int i = 0; i<4; i++){
        for (int j = 0; j<10; j++){
          file<<mppt_buffer[j][i]<<",";
        }
      file<<" ,";
      }
//      file<<"Driver,";
//      for(int i = 0; i<8; i++){
//        file<<(int)driver_state_data.data[i]<<",";
//      }
//      " ,";
//      for(int i = 0; i<8; i++){
//        file<<(int)reference_data.data[i]<<",";
//      }
//      " ,";
//      for(int i = 0; i<8; i++){
//        file<<(int)supply_1_data.data[i]<<",";
//      }
//      " ,";
//      for(int i = 0; i<8; i++){
//        file<<(int)motor_1_data.data[i]<<",";
//      }
//      " ,";      
//      for(int i = 0; i<8; i++){
//        file<<(int)motor_2_data.data[i]<<",";
//      }      
      file<<"\n"<<flush;
    } 
//    
//    if (counter%10 == 1){
//      
//      
//    }
//    
//     DataStore *m_PID_data;
//     DataStore *m_complementary_data;
//     DataStore *m_xsens_state_data;
//     
//     DataStore::PIDDataTotal log_OUTPUT_PID  = m_PID_data-> GetPIDData(); 
//     DataStore::RealData log_INPUT_PID = m_complementary_data-> GetComplementaryData();
//     DataStore::XsensData log_xsens = m_xsens_state_data->GetXsensData();
//     
//     control_file<<counter<<log_OUTPUT_PID.Force_height << ","<<log_OUTPUT_PID.Force_pitch << ","<<log_OUTPUT_PID.Force_roll << ","
//         <<log_INPUT_PID.Real_height << ","<<log_INPUT_PID.Real_pitch<< ","<<log_INPUT_PID.Real_roll<< ","
//         <<log_xsens.roll<< ","<<log_xsens.acceleration_z<< ","<<log_xsens.acceleration_x<<"\n"<<flush;
//     
//  
//    
  }
  return 0;
}