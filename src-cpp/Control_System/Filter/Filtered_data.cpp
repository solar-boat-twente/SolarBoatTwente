/* 
 * File:   RuwDataFilter.cpp
 */
#include <stdio.h>
#include <string>
#include <iostream>
#include <math.h>

#include "../../../lib-cpp/Debugging/easy_debugging.hpp"

#include "Filtered_data.h"


using namespace std;
using namespace MIO;
using namespace control;

// Constructor die geen file opent.
RawDataFilter::RawDataFilter(DataStore * const control_data,double Fc, double sample_rate) 
: control_data_(control_data), filtered_data_(new DataStore::FilteredData){

  // Create a Biquad, lpFilter.
    low_pass_filter = new Biquad(bq_type_highpass, Fc / sample_rate, 0.707, 0);  
    low_pass_filter->setBiquad(bq_type_highpass, Fc / sample_rate, 0.707, 0);
    
}

RawDataFilter::~RawDataFilter() {
  
  delete filtered_data_;
  delete low_pass_filter;
}

void RawDataFilter::filter_data() {
    // retreive data from control data
  DataStore::sensor_struct raw_data = control_data_->GetSensorData();



  M_INFO << "roll voor filter is  "<<raw_data.roll;
  M_INFO << "pitch voor filter is  "<<raw_data.pitch;
  float input_pitch = raw_data.pitch;
  float input_roll = raw_data.roll;
  float input_z_acceleration = raw_data.Z_accel;
  float input_left_angle = raw_data.angle_left;
  float input_right_angle = raw_data.angle_right;

  //Not yet filtering the data therefore filtered == raw data;
  filtered_data_->filtered_pitch = input_pitch;//sump/array;
  filtered_data_->filtered_roll = input_roll;//sumroll/array;
  filtered_data_->filtered_Z_accel = input_z_acceleration;//sumz/array;
  filtered_data_->filtered_angle_left = input_left_angle;//suml/array;
  filtered_data_->filtered_angle_right = input_right_angle;//sumr/array;
    
  M_INFO << "roll na filter is  "<<filtered_data_->filtered_roll;
  M_INFO << "pitch na filter is "<<filtered_data_->filtered_pitch;
  control_data_->PutFilteredData(filtered_data_);

}
