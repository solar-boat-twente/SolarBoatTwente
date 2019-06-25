/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "Motor.hpp"
#include <iostream>
#include "../../lib-cpp/Debugging/easy_debugging.hpp"

using namespace MIO;
using namespace PowerElectronics;

Motor::Motor(CANbus* driver_can, int read_flag): driver_can_(driver_can), read_flag_(read_flag){
  M_OK<<"MOTOR INITIALIZED";
}

void Motor::parse_motor_1(uint8_t data[]) {

  two_bytes raw_phase_current;
  two_bytes raw_link_voltage;
  raw_phase_current.bytes[1] = data[0];
  raw_phase_current.bytes[0] = data[1];
  raw_link_voltage.bytes[1] = data[2];
  raw_link_voltage.bytes[0] = data[3];
  
  if(values.Urange.signed_int == -1){
    M_WARN<<"CALCULATING VALUES WITHOUT USING REAL RANGES TAKING THE STANDARD RANGES!";
    values.phase_current = (float)raw_phase_current.signed_int/32768 * kMotorStandardIRange;
    values.link_voltage = (float)raw_link_voltage.signed_int/32678 * kMotorStandardURange;
  } else {
    values.phase_current = (float)raw_phase_current.signed_int/32768 * values.Irange.unsigned_int;
    values.link_voltage = (float)raw_link_voltage.signed_int/32678 * values.Urange.unsigned_int;
  }
  
}

void Motor::parse_motor_2(uint8_t data[]) {
  two_bytes raw_power;
  two_bytes raw_rpm;
  raw_power.bytes[1] = data[0];
  raw_power.bytes[0] = data[1];
  raw_rpm.bytes[1] = data[4];
  raw_rpm.bytes[0] = data[5];
  
  if(values.Urange.signed_int = -1){
    M_WARN<<"CALCULATING VALUES WITHOUT USING REAL RANGES TAKING THE STANDARD RANGES!";
    values.motor_power = (float)raw_power.signed_int/(32768) * kMotorStandardIRange * kMotorStandardURange;
  } else {
    values.motor_power = (float)raw_power.signed_int/(32768) * values.Irange.unsigned_int * values.Urange.unsigned_int;   
  }
  values.rotor_speed = 8 * raw_rpm.signed_int;
  
  if(values.rotor_speed!=0){
    values.torque = values.motor_power/values.rotor_speed*9.55; //Conversion from rpm to rad/s and then use tau = P/omega
  } else {
    values.torque = -1;
  }
  
    M_ERR<<"PARSING MOTOR 2 SUCCESSFULL: " << values.rotor_speed<<" "<<values.motor_power<<" "<<values.torque; 

}

void Motor::parse_supply_1(uint8_t data[]) {
  
  two_bytes raw_current;
  two_bytes raw_voltage;
  raw_current.bytes[1] = data[0];
  raw_current.bytes[0] = data[1];
  raw_voltage.bytes[1] = data[2];
  raw_voltage.bytes[0] = data[3];
  
  if(values.Urange.signed_int == -1){
    M_WARN<<"CALCULATING VALUES WITHOUT USING REAL RANGES TAKING THE STANDARD RANGES!";
    values.supply_current = raw_current.signed_int/32768 * kMotorStandardIRange;
    values.supply_voltage = raw_voltage.signed_int/32678 * kMotorStandardURange;
  } else {
    values.supply_current = raw_current.signed_int/32768 * values.Irange.unsigned_int;
    values.supply_voltage = raw_voltage.signed_int/32678 * values.Urange.unsigned_int;
  }
  
}

void Motor::parse_driver_data(uint8_t data[]) {
  values.high_priority_limiter = data[1];
  values.motor_mode = data[2];
  values.driver_temp = data[3];
  values.low_priority_limiter.bytes[0] = data[4];
  values.low_priority_limiter.bytes[1] = data[5];
  values.error_word.bytes[0] = data[6];
  values.error_word.bytes[1] = data[7];
}

void Motor::parse_reference(uint8_t data[]) {
  values.Urange.bytes[1] = data[0];
  values.Urange.bytes[0] = data[1];
  values.Irange.bytes[1] = data[2];
  values.Irange.bytes[0] = data[3];
  values.Uref.bytes[1] = data[4];
  values.Uref.bytes[0] = data[5];
  values.Iref.bytes[1] = data[6];
  values.Iref.bytes[0] = data[7];
}

bool Motor::update_data_from_driver() {
  canmsg_t buffer;
  watchdog = false;
  for (int i = 0; i<8; i++){
    
    if((read_flag_>>i)%2){
      M_INFO<<"TRUE FOR I = "<<i<<" read_flag_: "<<read_flag_;
      switch (i){
        case 0:
          if (driver_can_->read_can(CANID_MOTOR_SUPPLY_1, &buffer)==1){
            parse_supply_1(buffer.data);
            watchdog = true;
          } 
          break;
        case 1:
          if (driver_can_->read_can(CANID_MOTOR_MOTOR_1, &buffer)==1){
            parse_motor_1(buffer.data);
            watchdog = true;
          }
          break;
        case 2:
          if (driver_can_->read_can(CANID_MOTOR_MOTOR_2, &buffer)==1){
            parse_motor_2(buffer.data);
            watchdog = true;            
          }
          break;
        case 3:
          if (driver_can_->read_can(CANID_MOTOR_DRIVER_STATE, &buffer)==1){
            parse_driver_data(buffer.data);
            watchdog = true;
          }
          break;
        case 4:
          if (driver_can_->read_can(CANID_MOTOR_RANGE_REFERENCE, &buffer)==1){
            parse_reference(buffer.data);
            watchdog = true;
          }
          break;
        default:
          M_ERR<<"ERROR INVALID READ STATEMENT FOR I = "<<i;
          break; 
      }
    }  
  }
  return watchdog;
}

void Motor::setup_sampling() {
  canmsg_t driver_sample;
  driver_sample.id = 0xC7;
  driver_sample.length = 8;
  
  driver_sample.data[0] = 0x0;
  driver_sample.data[1] = 0x08;
  driver_sample.data[2] = 0;
  driver_sample.data[3] = 0xff;
  driver_sample.data[4] = 0x01;
  driver_sample.data[5] = 0xE8;
  driver_sample.data[6] = 0x0;
  driver_sample.data[7] = 0x10;

  driver_can_->write_can(driver_sample);
}




