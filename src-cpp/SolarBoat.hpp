/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SolarBoat.hpp
 * Author: Sander Oosterveld
 *
 * Created on June 7, 2019, 9:54 AM
 */

#ifndef SOLARBOAT_HPP
#define SOLARBOAT_HPP

#include "../solarboattwente.h"
#include "Battery_Magagement_System/BMS.h"
#include "Motor/Motor.hpp"
#include "Button_Box/Button_Box.hpp"
#include "Control_Wheel/Control_Wheel.hpp"
#include "Temperature_Sensor/Temperature_Sensor.hpp"
namespace MIO{
class SolarBoat{
 public:
  start();
  
 private:
  /*set the member structures and iniate some of the values to protect the sceens */
  void initiate_structures();
  
  void initiate_components();
  
  void initiate_screen();
  
  void blink_leds();
  
  void stop_leds();
  
  void update_leds();
  
  bool update_motor();
  
  void generate_driver_message(canmsg_t * driver_tx, int speed);
  
  int get_speed();
  
  double seconds_since_start(time_t start);
  
  void write_to_screen();
  
  structures::PowerInput * power_input;
  structures::PowerOutput * power_output;
  structures::UserInput * user_input;
  structures::ControlData * control_data;
  structures::TelemetryInput * telemetry_data;
  
  Serial * serial_wheel;
  CANbus * canbus_bms;
  CANbus * canbus_driver;
  UI::ADAM * adam_6050;
  
  
  
  
  
  
  
  
  
  
  
  
  
};
}

#endif /* SOLARBOAT_HPP */

