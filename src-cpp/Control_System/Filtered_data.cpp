/* 
 * File:   RuwDataFilter.cpp
 */
#include <stdio.h>
#include <string>
#include <iostream>
#include <math.h>
#include "Filtered_data.h"
#include "../../lib-cpp/Serial/Serial.h"
#include "Xsens.h"


using namespace std;
using namespace MIO;
using namespace xsens;

// Constructor die geen file opent.
RuwDataFilter::RuwDataFilter( ) {
    cout << "Called constructor RuwDataFilter()" << endl;
    double Fc=20;
    double sampleRate=80;
    // Create a Biquad, lpFilter.
    lpFilter = new Biquad(bq_type_highpass, Fc / sampleRate, 0.707, 0);  
    lpFilter->setBiquad(bq_type_highpass, Fc / sampleRate, 0.707, 0);
}

RuwDataFilter::~RuwDataFilter( ) {
   // cout << "Called destructor RuwDataFilter()" << endl;
}

void RuwDataFilter::FilterIt( ) {
    // Haal de data op.
    DataStore::sensor_struct ruw = m_ruwe_state_data->GetSensorData();
    //printf("xsens %f,%f" ,ruw.pitch,ruw.roll);
    //XsensData Xsens_data;
    //XsensData * DataStruct =&Xsens_data;
   
    DataStore::FilteredData filtereddata;
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

      cout << "roll voor filter is  "<<ruw.roll << "\r\n";
      cout << "pitch voor filter is  "<<ruw.pitch << "\r\n";
      inp = ruw.pitch;
      inroll = ruw.roll;
      inz = ruw.Z_accel;
      inl = ruw.angle_left;
      inr = ruw.angle_right;
//}
//    //for (int idx = 0; idx < array; idx++) {
//      cout << "roll voor filter is  "<<ruw.roll[idx] << "\r\n";
//      inp[idx] = ruw.pitch[idx];
//      cout << "pitch voor filter is  "<<ruw.pitch[idx] << "\r\n";
//      sump=inp[idx]+sump;
//      cout << "sumpitch heeft een waarde van " << sump << "\r\n";
//      inroll[idx] = ruw.roll[idx];
//      sumroll=inroll[idx]+sumroll;
//      cout << "sumroll heeft een waarde van " << sumroll << "\r\n";
//      inz[idx] = ruw.Z_accel[idx];
//      sumz=inz[idx]+sumz;
//  //    inl[idx] = lpFilter->process(ruw.angle_left[idx]);
//  //    suml=inl[idx]+suml;
//  //    inr[idx] = lpFilter->process(ruw.angle_right[idx]);
//  //    sumr=inr[idx]+sumr;
//      inl[idx] = ruw.angle_left[idx];
//      suml=inl[idx]+suml;
//      inr[idx] = ruw.angle_right[idx];
//      sumr=inr[idx]+sumr;
//    }
    
  filtereddata.filtered_pitch = inp;//sump/array;
  filtereddata.filtered_roll = inroll;//sumroll/array;
  filtereddata.filtered_Z_accel = inz;//sumz/array;
  filtereddata.filtered_angle_left = inl;//suml/array;
  filtereddata.filtered_angle_right = inr;//sumr/array;
    
    cout << "roll na filter is  "<<filtereddata.filtered_roll << "\r\n" ;
    cout << "pitch na filter is "<<filtereddata.filtered_pitch  << "\r\n";
    m_filtered_data->PutFilteredData(&filtereddata);

}
