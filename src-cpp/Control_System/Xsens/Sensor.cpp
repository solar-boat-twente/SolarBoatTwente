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

xsens::Sensor::~Sensor(){
}

void xsens::Sensor::get_data(){
  //get data from xsens
  DataStore::XsensData x = xsens_state_data_->GetXsensData();
  M_OK<<"GOTTEN DATA FROM XSENS: CHECK ROLL EQUAL TO: "<<x.roll;
  
  
  //Convert sensor_values to radials and write it to raw_data Datastore object
  sensor_value->pitch = x.pitch/180*3.1415;        //0.35 rad = 20 graden 
  sensor_value->roll = x.roll/180*3.1415;        
  sensor_value->Z_accel =x.acceleration_z; 
  sensor_value->angle_left = 0;   //0.52 rad = 30 graden
  sensor_value->angle_right = 0; 
  raw_data_->PutSensorData(sensor_value);
}