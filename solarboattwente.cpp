/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <thread>

#include "solarboattwente.h"
#include "lib-cpp/Canbus/canbus.h"

using namespace MIO;
using namespace structures;

PowerOutputHandler::PowerOutputHandler(PowerInput* p_i, PowerOutput* p_o, UserInput* u_i): power_input(p_i), power_output(p_o), user_input(u_i){
}

int PowerOutputHandler::handling_function(short int delay) {
  while(active){
    if(user_input->buttons.battery_on){
      power_output->contractor_control = 1; //Connect the contactor if the BMS wants to
    } else {
      power_output->contractor_control = 0; //Disconnect the contactor
      power_output->balancing_control = 0;
    }
    //When the force battery button and battery on button are pressed the battery is forced
    if (user_input->buttons.force_battery && user_input->buttons.battery_on){
      power_output->contractor_control = 2; //Force the contactor to be connected
      power_output->balancing_control = 1; //Force the balancing off
    }  else {
      power_output->balancing_control = 0;
    }

    //Switch all the MPPTs whether they should be on.
    for(int i = 0; i<10; i++){
      power_output->solar_panel_states[i] = user_input->buttons.solar_on;
    } 
  }
  return 1;
}

int PowerOutputHandler::start(short int delay) {
  active = true;
  m_thread = std::thread(&PowerOutputHandler::handling_function, this, delay);
  return 1;
}

int PowerOutputHandler::stop(){
  active = false;
   m_thread.join();
  return 1;
}
