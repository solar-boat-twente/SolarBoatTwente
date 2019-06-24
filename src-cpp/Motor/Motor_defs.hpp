/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Motor_CANID.hpp
 * Author: Sander Oosterveld
 *
 * Created on May 23, 2019, 12:10 PM
 */

#ifndef MOTOR_CANID_HPP
#define MOTOR_CANID_HPP

namespace MIO{
namespace PowerElectronics{
//TX messages
const short int CANID_MOTOR_SAMPLE = 0xc7;
const short int CANID_MOTOR_DRIVER_CMD = 0xcf;

//RX message
const short int CANID_MOTOR_DRIVER_STATE = 0xd0;
const short int CANID_MOTOR_RANGE_REFERENCE = 0xd8;
const short int CANID_MOTOR_SUPPLY_1 = 0xe0;
const short int CANID_MOTOR_MOTOR_1 = 0xf0;
const short int CANID_MOTOR_MOTOR_2 = 0xf8;

const short int STD_MOTOR_READ_DELAY = 500;
const short int STD_MOTOR_SAMPLE_PERIOD = 750;
const short int MOTOR_OFF_TIMEOUT = 3000;

struct Supply1Data{
  short int supply_current_relative;
  float supply_current; //Amps
  short int supply_voltage_relative;
  float supply_voltage; //Volt
  int total_charge; //mAh
};

struct Motor1Data{
  short int phase_current_relative;
  float phase_current; //Amps
  short int link_voltage_relative;
  float link_voltage; //Volt
  int total_energy; // Joule
};

struct Motor2Data{
  short int motor_power_relative;
  float motor_power; //Joule
  short int rotor_speed_relative;
  float rotor_speed; //RPM
  float torque; //Nm
  signed short int revolutions; //number of revolutions
};

struct DriverStateData{
  uint8_t first_byte;
  bool motor_driven;
  uint8_t motor_driver_algorithm; //1 is BLDC, 2 is Vector, 0 is invalid value;
  uint8_t high_priority_limiter; //Look at driver manual --> should be 0
  uint8_t motor_mode; //Value between 0 and 10; --> should be 2
  uint8_t driver_temp; //degrees Celsius;
  short int low_priority_limiter; //Look at driver manual --> should be 0 
  short int error_word; //Look at driver manual --> should be 0
};

struct RangeReferenceData{
  short int Urange; //Volts
  short int Irange; //Amperes
  
  short int Uref; //Volts
  short int Iref; //Amperes
};


const int RD_SUPPLY1 = 1<<0;
const int RD_MOTOR1 = 1<<1;
const int RD_MOTOR2 = 1<<2;
const int RD_DRIVERSTATE = 1<<3;
const int RD_RANGEREF = 1<<4;


}
}

#endif /* MOTOR_CANID_HPP */

