/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "ConfigReader.hpp"

using namespace libconfig;
using namespace MIO;

ConfigReader::ConfigReader(const std::string& config_file_name) {
  cfg_.readFile(config_file_name);
  
}

float ConfigReader::get_float_data(ConfigFloats config) {
  const Setting& root = cfg_.getRoot();
  const Setting& control = root["control"];
  const Setting& vlotter = root["vlotter"];

  float value;
  switch (config) {
    case ConfigFloats::HEIGHT_SET_POINT:
      control.lookupValue("height_set_point", value);
      return value;
      
    case ConfigFloats::LIFT_OF_SPEED:
      control.lookupValue("lift_of_speed", value);
      return value;
      
    case ConfigFloats::ROLL_START_SPEED:
      control.lookupValue("roll_start_speed", value);
      return value;
      
    case ConfigFloats::VLOTTER_LEFT:
      vlotter.lookupValue("zero_angle_left", value);
      return value;
      
    case ConfigFloats::VLOTTER_RIGHT:
      vlotter.lookupValue("zero_angle_right", value);
      return value;
  }
}

int ConfigReader::get_int_data(ConfigInts config) {
  const Setting& root = cfg_.getRoot();
  const Setting& control = root["control"];
  const Setting& power = root["power"];
  const Setting& control_states = control["pid_states"];
  
  int value;
  switch (config){
    case ConfigInts::HEIHGT_STATE:
      control_states.lookupValue("height", value);
      return value;
      
    case ConfigInts::ROLL_STATE:
      control_states.lookupValue("roll", value);
      return value;
      
    case ConfigInts::LIFT_OF_FORCE:
      control.lookupValue("lift_of_force", value);
      return value;
      
    case ConfigInts::THROTTLE_SWITCH:
      power.lookupValue("throttle_switch", value);
      return value;
  }
}

std::vector<int> ConfigReader::get_height_states() {
  const Setting& root = cfg_.getRoot();
  const Setting& pid_roll = root["control"]["pid_roll"];
  
  std::vector<int> output;
  output.push_back(pid_roll[0]);
  output.push_back(pid_roll[1]);
  output.push_back(pid_roll[2]);
  
  return output;
}

std::vector<int> ConfigReader::get_roll_states() {
  const Setting& root = cfg_.getRoot();
  const Setting& pid_roll = root["control"]["pid_height"];
  
  std::vector<int> output;
  output.push_back(pid_roll[0]);
  output.push_back(pid_roll[1]);
  output.push_back(pid_roll[2]);
  
  return output;
}



