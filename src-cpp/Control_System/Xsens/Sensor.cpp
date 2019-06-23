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



xsens::Sensor::Sensor(DataStore * const control_data, control::Vlotter * const vlotter)
: control_data_(control_data), 
    vlotter_(vlotter), 
    sensor_data_(new DataStore::sensor_struct)
{
  
}

xsens::Sensor::Sensor(const Sensor& other) 
: control_data_(other.control_data_),
  vlotter_(other.vlotter_),
    sensor_data_(new DataStore::sensor_struct)
{}

xsens::Sensor xsens::Sensor::operator=(const Sensor& other) {
  return *this;
}

xsens::Sensor::~Sensor() {
  delete sensor_data_;
}

void xsens::Sensor::update_data(){
  //get data from xsens
  DataStore::XsensData xsens_data = control_data_->GetXsensData();
  M_OK<<"GOTTEN DATA FROM XSENS: CHECK ROLL EQUAL TO: "<<xsens_data.roll;
  
  
  
  //Convert sensor_values to radials and write it to raw_data Datastore object
  sensor_data_->pitch = xsens_data.pitch/180*3.1415;        //0.35 rad = 20 graden 
  sensor_data_->roll = xsens_data.roll/180*3.1415;        
  sensor_data_->Z_accel =xsens_data.acceleration_z; 
  sensor_data_->angle_left = vlotter_->get_angle_rad(control::EncoderNumber::ENCODER_LEFT);   //0.52 rad = 30 graden
  sensor_data_->angle_right = vlotter_->get_angle_rad(control::EncoderNumber::ENCODER_RIGHT); 
  
  control_data_->PutSensorData(sensor_data_);
}