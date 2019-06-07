///////////////////////////////////////////////////////////
//  sensor.cpp
//  Implementation of the Class sensor
//  Created on:      20-Feb-2019 11:56:45
//  Original author: wiegmink
///////////////////////////////////////////////////////////
#include <cstdlib>
#include <string>
#include <stdio.h>
#include "Sensor.h"
#include "../../lib-cpp/Serial/Serial.h"
#include "../../lib-cpp/Canbus/canbus.h"
//#include "structures.h"
#include <math.h>
#include "Xsens.h"
#include <iostream>
#include <mutex>

using namespace std;
using namespace MIO;
using namespace xsens;

mutex mutexje_getdata;

Sensor::Sensor():m_ruwe_state_data(NULL), data_is_op(false) {
}

Sensor::~Sensor(){
}

void Sensor::get_data(){
  int array = 1;
  //for (int i = 0; i < array; i++){
  mutexje_getdata.lock();
  DataStore::XsensData x = m_xsens_state_data->GetXsensData();
  mutexje_getdata.unlock();
  
  cout << "roll is "<<x.roll << "\r\n";
  DataStore::sensor_struct sensor_waarde;

  sensor_waarde.pitch = x.pitch/180*3.1415;        //0.35 rad = 20 graden 
  sensor_waarde.roll = x.roll/180*3.1415; 
//        sensor_waarde.pitch[j]=0;
//        sensor_waarde.roll[j]=0.7;        
  sensor_waarde.Z_accel =x.acceleration_z; 
  sensor_waarde.angle_left = 0;   //0.52 rad = 30 graden
  sensor_waarde.angle_right = 0; 
//       this_thread::sleep_for(chrono::milliseconds(1));

    m_ruwe_state_data->PutSensorData(&sensor_waarde);
    
   // cout << "waarde van j is " << j << "\r\n";
    //if (j<4){
    //  j++;
    //}
    //else {
    //  j=0;
    //}
}