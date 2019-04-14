/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: Sander Oosterveld
 *
 * Created on April 9, 2019, 7:22 PM
 */
#include <cstdlib>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <thread>
#include "../src/Wrappers/canbus.h"
#include "../src/structures.h"
#include "../src/Battery_Magagement_System/BMS.h"
#include "../src/Battery_Magagement_System/BMS_CANIDs.h"

#include "../include/easy_debugging.hpp"



using namespace std;
using namespace MIO;
/*
 * 
 */
int main(){
  char variable = 0x59;
  M_INFO<<"Hello";
  M_INFO<<"INFO MESSAGE";
  M_ERR<<"ERROR MESSAGE";
  M_WARN<<"WARNING "<<CANID_BMS_RESPONSE1<<variable;
  M_OK<<"OK MESSAGE";
  
  CANbus * canbus = new CANbus("can0");
  structures::PowerInput * power_input = new structures::PowerInput;
  PowerElectronics::BMS * bms = new PowerElectronics::BMS(canbus);
  canbus->start(10000);
  bms->start_reading(power_input);
  return 0;
}






/*
int main(int argc, char** argv) {
  char response1[] = {80, 97, 168, 0, 0, 178, 2, 5};
  char response2[] = {60,30,1,0,1};
  char response3[] = {11, 184, 15, 160, 19, 136, 23, 112};
  
  CANbus * const canbus = new CANbus("can1");
  structures::PowerInput * const power_input = new structures::PowerInput;
  canmsg_t *message1 = new canmsg_t;
  canmsg_t *message2 = new canmsg_t;
  canmsg_t *message3 = new canmsg_t;
  canmsg_t *message4 = new canmsg_t;
  message1->id = CANID_BMS_RESPONSE1;
  message1->length = 8;
  for(int i=0; i<message1->length;i++){
    message1->data[i] = response1[i];
  }
  
  message2->id = CANID_BMS_RESPONSE2;
  message2->length = 5;
  for(int i=0; i<message2->length; i++){
    message2->data[i] = response2[i];
  }
      
  message3->id = CANID_BMS_CELL_VOLTAGE_START;
  message3->length = 8;
  for(int i=0; i<message3->length;i++){
    message3->data[i] = response3[i];
  }
  
  *message4 = *message3;
  
  message4->id=CANID_BMS_CELL_VOLTAGE_START+1;
  
  canbus->add_message_(message1);
  canbus->add_message_(message2);
  canbus->add_message_(message3);
  canbus->add_message_(message4);
  
  PowerElectronics::BMS * const bms = new PowerElectronics::BMS(canbus);
  bms->start_reading(power_input);
  this_thread::sleep_for(chrono::milliseconds(3000));
  
  cout<<INFO("\n============================================= OUTPUT DATA ==========================================")<<endl;
  fprintf(stderr,INFO("soc: %f\tvoltage: %f\tcurrent: %f\terror: %i\t error_loc: %i\n"),
      power_input->battery.state_of_charge, power_input->battery.total_voltage, power_input->battery.total_current,
      power_input->battery.error_number, power_input->battery.error_location);
  fprintf(stderr, INFO("Max_temp: %i\tMin_temp: %i\tBalance: %i\tCont_ready: %i\tCont_stat: %i\n"),
          power_input->battery.max_temp, power_input->battery.min_temp, power_input->battery.balance_state,
          power_input->battery.contactor_ready, power_input->battery.contactor_status);
  for(int i = 0; i<12; i++){
    printf(INFO("Cell %i: %f\t"), i+1, power_input->battery.cel_voltages[i]);
    if(i%4==3){
      printf("\n");
    }
  }    
  cout<<INFO("====================================================================================================")<<endl;

  return 0;
}*/

