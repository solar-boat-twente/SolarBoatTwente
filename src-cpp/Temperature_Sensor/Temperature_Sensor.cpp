/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <fstream>

#include "Temperature_Sensor.hpp"

using namespace MIO;
using namespace std;



float TemperatureSensor::get_core_temp_1() {
  string output;
  ifstream file;
  file.open(CORE_1_PATH);
  getline(file, output);
  
  return (float)stoi(output)/1000;
}

float TemperatureSensor::get_core_temp_2() {
  string output;
  ifstream file;
  file.open(CORE_2_PATH);
  getline(file, output);
  
  return (float)stoi(output)/1000;
}

float TemperatureSensor::get_temp_zone() {
  string output;
  ifstream file;
  file.open(TEMP_ZONE_PATH);
  getline(file, output);
  
  return (float)stoi(output)/1000;
}