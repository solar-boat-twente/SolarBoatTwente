/* 
 * File:   RuwDataFilter.cpp
 */
#include <stdio.h>
#include <string>
#include <iostream>
#include <math.h>
#include "Filtered_data.h"
#include "Serial.h"
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
    int array = 4;
    float inp[array];
    float inroll[array];
    float inz[array];
    float inl[array];
    float inr[array];
    float sump=0;
    float sumroll=0;
    float sumz=0;
    float suml=0;
    float sumr=0;
  
    for (int idx = 0; idx < array; idx++) {
    cout << "roll voor filter is  \r\n"<<ruw.roll[idx] ;
    inp[idx] = ruw.pitch[idx];
    sump=inp[idx]+sump;
    inroll[idx] = ruw.roll[idx];
    sumroll=inroll[idx]+sumroll;
    inz[idx] = ruw.Z_accel[idx];
    sumz=inz[idx]+sumz;
//    inl[idx] = lpFilter->process(ruw.angle_left[idx]);
//    suml=inl[idx]+suml;
//    inr[idx] = lpFilter->process(ruw.angle_right[idx]);
//    sumr=inr[idx]+sumr;
    inl[idx] = ruw.angle_left[idx];
    suml=inl[idx]+suml;
    inr[idx] = ruw.angle_right[idx];
    sumr=inr[idx]+sumr;
    }
    
//    for (int idx = 0; idx < 4; idx++) {
//    //cout << "roll voor filter is  \r\n"<<ruw.roll[idx] ;
//    inp[idx] = lpFilter->process(ruw.pitch[idx]);
//    sump=inp[idx]+sump;
//    inroll[idx] = lpFilter->process(ruw.roll[idx]);
//    sumroll=inroll[idx]+sumroll;
//    inz[idx] = lpFilter->process(ruw.Z_accel[idx]);
//    sumz=inz[idx]+sumz;
//    inl[idx] = lpFilter->process(ruw.angle_left[idx]);
//    suml=inl[idx]+suml;
//    inr[idx] = lpFilter->process(ruw.angle_right[idx]);
//    sumr=inr[idx]+sumr;
//    }
    //printf("filtered data = %f,%f,%f,%f,%f",filtereddata.filtered_pitch,filtereddata.filtered_roll,filtereddata.filtered_Z_accel, filtereddata.filtered_angle_left, filtereddata.filtered_angle_right);
    // En bewaar het voor de volgende ronde.
   //cout << "roll na filter is  \r\n"<<filtereddata.filtered_roll  ;
    //m_filtered_data->PutSensorData(&ruw);
  filtereddata.filtered_pitch = sump/array;
  filtereddata.filtered_roll = sumroll/array;
  filtereddata.filtered_Z_accel = sumz/array;
  filtereddata.filtered_angle_left = suml/array;
  filtereddata.filtered_angle_right = sumr/array;
    
    cout << "roll na filter is  \r\n"<<filtereddata.filtered_roll  ;
    cout << "pitch na filter is  \r\n"<<filtereddata.filtered_pitch  ;
    m_filtered_data->PutFilteredData(&filtereddata);

}

