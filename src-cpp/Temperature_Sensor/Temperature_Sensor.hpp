/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Temperature_Sensor.hpp
 * Author: Sander Oosterveld
 *
 * Created on June 6, 2019, 9:17 AM
 */

#ifndef TEMPERATURE_SENSOR_HPP
#define TEMPERATURE_SENSOR_HPP

#include <string>

namespace MIO{


const std::string CORE_1_PATH = "/sys/class/hwmon/hwmon1/temp2_input"; 
const std::string CORE_2_PATH = "/sys/class/hwmon/hwmon1/temp4_input"; 
const std::string TEMP_ZONE_PATH = "/sys/class/hwmon/hwmon0/temp1_input"; 




class TemperatureSensor{
  
 public:
  TemperatureSensor(){
    
  };
  
  float get_core_temp_1();
  
  float get_core_temp_2();
  
  float get_temp_zone();
  
  
};





}

#endif /* TEMPERATURE_SENSOR_HPP */

