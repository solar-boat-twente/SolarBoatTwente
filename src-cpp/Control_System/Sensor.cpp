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
#include "Serial.h"
#include "canbus.h"
#include "structures.h"
#include <math.h>
#include "Xsens.h"
#include <iostream>

using namespace std;
using namespace MIO;
using namespace xsens;


Sensor::Sensor():m_ruwe_state_data(NULL), data_is_op(false) {

  //  cout << "Called constructor sensor() for test from input file" << endl;
}

Sensor::~Sensor(){

   // cout << "Called destructor sensor()" << endl;
}


void Sensor::get_data(){
  
//  //wait 500 milliseconds, because otherwise the xsens can enter the configuration mode
//  std::this_thread::sleep_for(std::chrono::milliseconds(500));
//
//  //define all bytes you are going to read. 
    uint8_t msg_bytes[35];
  
    //int j=0;     
    int array=4;
    //serie->read_bytes(msg_bytes, 35);
    //while ( m<35) {
      
//        if (msg_bytes[m] == 0xFA){
    uint8_t msg[35];
    uint8_t msg_[350];
    serie->read_bytes(msg_, 350);
    //serie->read_bytes(msg_bytes, 35);
    //serie->read_bytes(msg_bytes, 35);

        for (int m = 0; m < 35; m++) {
            msg[m]=msg_[m];
        }
        sens->ParseMessage(msg);
        sens->ParseData();
        DataStore::XsensData x = m_xsens_state_data->GetXsensData();
        cout << "roll is "<<x.roll ;
        DataStore::sensor_struct sensor_waarde;
        cout << "Called sensor::get_data(): " ;
        sensor_waarde.pitch[j] = x.pitch/180*3.1415;        //0.35 rad = 20 graden 
        sensor_waarde.roll[j] = x.roll/180*3.1415; 
//        sensor_waarde.pitch[j]=0;
//        sensor_waarde.roll[j]=0.7;        
        sensor_waarde.Z_accel[j] =x.acceleration_z; 
        sensor_waarde.angle_left[j] = 0;   //0.52 rad = 30 graden
        sensor_waarde.angle_right[j] = 0; 
 //       this_thread::sleep_for(chrono::milliseconds(1));
        m_ruwe_state_data->PutSensorData(&sensor_waarde);
        //k=k+1;
        j++;
        cout << "j is"<< j;
            if (j==array){
            j=0;
            
            }
        
              
    }
        //m=35;


//        else{
//        m++;
//        }
        //}   
//    sens->ParseMessage(msg_bytes);
//    sens->ParseData();
//    DataStore::XsensData x = m_xsens_state_data->GetXsensData();
//    cout << "roll is "<<x.roll ;
//    //DataStore::sensor_struct sensor_waarde;
//    cout << "Called sensor::get_data(): " ;
//    sensor_waarde.pitch[k] = (x.pitch/180)*3.1415;        //0.35 rad = 20 graden 
//    sensor_waarde.roll[k] = (x.roll/180)*3.1415; 
//    sensor_waarde.Z_accel[k] =0; 
//    sensor_waarde.angle_left[k] = 0;   //0.52 rad = 30 graden
//    sensor_waarde.angle_right[k] = 0; 
//    m_ruwe_state_data->PutSensorData(&sensor_waarde);
    
    //printf("sensor is: %f,%f,%f,%f,%f\r\n",margriet->pitch, margriet->roll, margriet->Z_accel, margriet->angle_left, margriet->angle_right);
    //printf("xsens = %f \r\n",sens->pitch);
    //cout << "roll is "<<sensor_waarde.roll[k] ;

    //cout << "roll array is "<<sensor_waarde.roll ;

//
//bool Sensor::data_op(){
//    return data_is_op;
//}