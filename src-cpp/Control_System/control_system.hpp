/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   control_system.hpp
 * Author: Danielle Kruijver
 *
 * Created on 11 juni 2019, 10:27
 */

#ifndef CONTROL_SYSTEM_HPP
#define CONTROL_SYSTEM_HPP

class ControlSystem{
 public: 
  bool enable_homing_maxon1(); //left, in this function homing is automatically enabled and directly started
  bool enable_homing_maxon2(); //right, in this function homing is automatically enabled and directly started
  bool enable_homing_maxon4(); //back, in this function homing is automatically enabled and directly started
  
  bool enable_profile_position_mode_maxon1(); //true: profile position mode is enabled and directly started
  bool enable_profile_position_mode_maxon2(); //true: profile position mode is enabled and directly started
  bool enable_profile_position_mode_maxon4(); //true: profile position mode is enabled and directly started
  
  bool start_homing();
  bool start_position_mode();
  
  start_logging(int frequency = 10);
  state_pid_roll(int state); //the right case has to be given as the input 
  state_pid_height(int state); //the right case has to be given as the input
  
  bool enable_xsens(); //start the xsens thread
    
      
};

#endif /* CONTROL_SYSTEM_HPP */