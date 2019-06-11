/* 
 * File:   RuwDataFilter.cpp
 */
#include <stdio.h>
#include <string>
#include <iostream>
#include <math.h>

#include "../../lib-cpp/Debugging/easy_debugging.hpp"

#include "Filtered_data.h"


using namespace std;
using namespace MIO;
using namespace Control;

// Constructor die geen file opent.
RawDataFilter::RawDataFilter(double Fc, double sample_rate) {
    cout << "Called constructor RuwDataFilter()" << endl;

    // Create a Biquad, lpFilter.
    low_pass_filter = new Biquad(bq_type_highpass, Fc / sample_rate, 0.707, 0);  
    low_pass_filter->setBiquad(bq_type_highpass, Fc / sample_rate, 0.707, 0);
    
    m_filtered_data_ = new DataStore::FilteredData;
}

RawDataFilter::~RawDataFilter() {
   // cout << "Called destructor RuwDataFilter()" << endl;
}

void RawDataFilter::filter_data() {
    // Haal de data op.
  DataStore::sensor_struct ruw = raw_state_data_->GetSensorData();

  // Filter het.
  int array = 1;
  float inp;
  float inroll;
  float inz;
  float inl;
  float inr;
//    float inp[array];
//    float inroll[array];
//    float inz[array];
//    float inl[array];
//    float inr[array];
  float sump=0;
  float sumroll=0;
  float sumz=0;
  float suml=0;
  float sumr=0;

  M_INFO << "roll voor filter is  "<<ruw.roll;
  M_INFO << "pitch voor filter is  "<<ruw.pitch;
  inp = ruw.pitch;
  inroll = ruw.roll;
  inz = ruw.Z_accel;
  inl = ruw.angle_left;
  inr = ruw.angle_right;

  //Not yet filtering the data therefore filtered == raw data;
  m_filtered_data_.filtered_pitch = inp;//sump/array;
  m_filtered_data_.filtered_roll = inroll;//sumroll/array;
  m_filtered_data_.filtered_Z_accel = inz;//sumz/array;
  m_filtered_data_.filtered_angle_left = inl;//suml/array;
  m_filtered_data_.filtered_angle_right = inr;//sumr/array;
    
  M_INFO << "roll na filter is  "<<m_filtered_data_.filtered_roll;
  M_INFO << "pitch na filter is "<<m_filtered_data_.filtered_pitch;
  filtered_data_->PutFilteredData(m_filtered_data_);

}
