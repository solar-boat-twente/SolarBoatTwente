/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ConfigReader.hpp
 * Author: Sander Oosterveld
 *
 * Created on June 24, 2019, 2:24 PM
 */

#ifndef CONFIGREADER_HPP
#define CONFIGREADER_HPP

#include <string>
#include <vector>
#include <libconfig.h++>

constexpr auto kStandardConfigFileName = "/root/SolarBoat2019/SolarBoatTwente/config/SolarBoat.cfg";

namespace MIO{

enum class ConfigFloats{
  LIFT_OF_SPEED,
  ROLL_START_SPEED,
  HEIGHT_SET_POINT,
  VLOTTER_LEFT,
  VLOTTER_RIGHT,
  ZERO_LIFT_ANGLE
};

enum class ConfigInts{
  ROLL_STATE,
  HEIHGT_STATE,
  LIFT_OF_FORCE,
  THROTTLE_SWITCH
};

class ConfigReader{
  
 public:
  ConfigReader(const std::string& config_file_name = kStandardConfigFileName);
  
  std::vector<int> get_roll_states();

  std::vector<int> get_height_states();
  
  //returns the set state of the two wings order: a left, b left a right b right
  std::vector<float> get_function_parameters();

  float get_float_data(ConfigFloats config);
  
  int get_int_data(ConfigInts config);
  
 private:
  
  std::vector<int> config_list_to_vector_(libconfig::Setting& list);
  
  std::string file_name;
  libconfig::Config cfg_;
};
}
#endif
