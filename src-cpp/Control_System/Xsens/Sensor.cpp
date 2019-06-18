///////////////////////////////////////////////////////////
//  sensor.cpp
//  Implementation of the Class sensor
//  Created on:      20-Feb-2019 11:56:45
//  Original author: wiegmink
///////////////////////////////////////////////////////////
#include <cstdlib>
#include <string>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <mutex>

#include "../../../lib-cpp/Debugging/easy_debugging.hpp"

#include "Sensor.h"

using namespace std;
using namespace MIO;

xsens::Sensor::Sensor():raw_data_(NULL), data_is_op(false) {
}

xsens::Sensor::Sensor(DataStore * xsens_state_data, DataStore * raw_data, Control::Vlotter * vlotter)
: xsens_state_data_(xsens_state_data), 
  raw_data_(raw_data), 
  vlotter_(vlotter), 
  sensor_value(new DataStore::sensor_struct)
{}

xsens::Sensor::Sensor(const Sensor& other) 
: xsens_state_data_(other.xsens_state_data_),
  raw_data_(other.raw_data),
  vlotter_(other.vlotter),
  sensor_value(new DataStore::sensor_struct(other.sensor_value))
{}

Sensor xsens::Sensor::operator=(const Sensor& other) {
  return Sensor(other);
}

xsens::Sensor::~Sensor() {
  delete sensor_struct;
}

void xsens::Sensor::get_data(){
  //get data from xsens
  DataStore::XsensData x = xsens_state_data_->GetXsensData();
  M_OK<<"GOTTEN DATA FROM XSENS: CHECK ROLL EQUAL TO: "<<x.roll;
  
  
  //Convert sensor_values to radials and write it to raw_data Datastore object
  sensor_value->pitch = x.pitch/180*3.1415;        //0.35 rad = 20 graden 
  sensor_value->roll = x.roll/180*3.1415;        
  sensor_value->Z_accel =x.acceleration_z; 
  sensor_value->angle_left = vlotter_->get_angle_deg(Control::ENCODER_LEFT);   //0.52 rad = 30 graden
  sensor_value->angle_right = vlotter_->get_angle_deg(Control::ENCODER_RIGHT); 
  raw_data_->PutSensorData(sensor_value);
}