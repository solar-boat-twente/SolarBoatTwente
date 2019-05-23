/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: kwieg
 *
 * Created on February 28, 2019, 7:54 PM
 */
#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <cstdlib>
#include <math.h>
#include <chrono>
#include <iostream>
#include <ctime>
#include <ratio>

#include "Sensor.h"
#include "Filtered_data.h"
#include "DataStore.h"
#include "ComplementaryFilter.h"
#include "PID_caller.h"
#include "Force_to_wing_angle.h"
#include "Daan_Test1_maxon.h"
#include "canbus.h"
#include "structures.h"

//DigitalIn button(USER_BUTTON);
using namespace std;
using namespace std::chrono;
using namespace MIO;
using namespace structures;
using namespace xsens;
using namespace control;
//EPOS epos1(1);
//EPOS epos2(2);
//EPOS epos4(4);

//Thread thread1;
//Thread thread2;
//Thread thread4;
/* -----------------------------------------------------------------------------
Main
----------------------------------------------------------------------------- */

int c_main(int argc, char** argv) {
typedef std::chrono::microseconds ms;
typedef std::chrono::milliseconds mls;
    // Maak de objecten.
    DataStore * xsens_data = new DataStore();
    DataStore * ruwe_data = new DataStore();
    DataStore * filtered_data = new DataStore();
    DataStore * complementary_data= new DataStore();
    DataStore * pid_data = new DataStore();
    DataStore * FtoW_data = new DataStore();
//    Serial * m_serial = new Serial("/dev/xsense", 9600);
//    Xsens * m_xsens = new Xsens(); //initialise class Xsens
//    Sensor * de_sensor = new Sensor(m_xsens,m_serial);
    RuwDataFilter * filter = new RuwDataFilter();
    ComplementaryFilter * com_filter = new ComplementaryFilter();
    PID_caller * PID = new PID_caller();
    control::ForceToWingAngle * FtoW = new control::ForceToWingAngle();
    CANbus * canbus1 = new CANbus("can1", 1);
    canbus1->start(10);
    EPOS * maxon1 = new EPOS(canbus1,1);
    EPOS * maxon2 = new EPOS(canbus1,2);
    EPOS * maxon4 = new EPOS(canbus1,4);
//    MOVE * moving = new MOVE(maxon1,maxon2,maxon4);
    
    // De associaties, de verbindingen tussen de objecten.
//    m_xsens->m_xsens_state_data = xsens_data; //(0)
//    de_sensor->m_xsens_state_data = xsens_data;
//    de_sensor->m_ruwe_state_data = ruwe_data;  // (1)
//    filter->m_ruwe_state_data = ruwe_data;     // (2)
//    filter->m_filtered_data = filtered_data;        // (3)
//    com_filter->m_filtered_data = filtered_data; //(4)
//    com_filter->m_complementary_data = complementary_data; //(5)
//    PID->m_complementary_data = complementary_data;  //(6)
//    PID->m_PID_data = pid_data; //(7)
//    FtoW->m_complementary_data = complementary_data; //(8)
//    FtoW->m_PID_data = pid_data; //(9)
//    FtoW->m_FtoW_data = FtoW_data; //(10)
//    maxon1->m_FtoW_data = FtoW_data; //(11)
//    maxon2->m_FtoW_data = FtoW_data; //(12)
//    maxon4->m_FtoW_data = FtoW_data; //(13)
//    
/* -----------------------------------------------------------------------------
All three motors are going to home.
----------------------------------------------------------------------------- */    
    
//high_resolution_clock::time_point t0= high_resolution_clock::now();
    this_thread::sleep_for(chrono::milliseconds(10000));
    maxon1->Homing();
    this_thread::sleep_for(chrono::milliseconds(1000));
    maxon1->HomingCheck();
    
    this_thread::sleep_for(chrono::milliseconds(10000));
    maxon2->Homing();
    this_thread::sleep_for(chrono::milliseconds(1000));
    maxon2->HomingCheck();

    this_thread::sleep_for(chrono::milliseconds(10000));   
    maxon4->Homing();
    this_thread::sleep_for(chrono::milliseconds(1000));
    maxon4->HomingCheck();
    //wait(20);
    
/* -----------------------------------------------------------------------------
All three motors are going in the startpositionmode.
----------------------------------------------------------------------------- */    

    maxon1->StartPositionMode();
    this_thread::sleep_for(chrono::milliseconds(1000));     //was 1000 

    maxon2->StartPositionMode();
    this_thread::sleep_for(chrono::milliseconds(1000));

    maxon4->StartPositionMode();
    this_thread::sleep_for(chrono::milliseconds(10000));
    

 // wait 500 milliseconds, because otherwise the xsens can enter the configuration mode

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
////    uint8_t msg_[350];
////    m_serial->read_bytes(msg_,350);
//    high_resolution_clock::time_point t_1= high_resolution_clock::now();
//    mls s = std::chrono::duration_cast<mls>(t_1-t0);
//    int t_total = 26000-s.count();
//    printf("loop tijd is %i",t_total);
//    this_thread::sleep_for(chrono::milliseconds(t_total));
//    Serial * m_serial = new Serial("/dev/xsense", 9600);
//    Xsens * m_xsens = new Xsens(); //initialise class Xsens
//    Sensor * de_sensor = new Sensor(m_xsens,m_serial);
    
    filter->m_ruwe_state_data = ruwe_data;     // (2)
    filter->m_filtered_data = filtered_data;        // (3)
    com_filter->m_filtered_data = filtered_data; //(4)
    com_filter->m_complementary_data = complementary_data; //(5)
    PID->m_complementary_data = complementary_data;  //(6)
    PID->m_PID_data = pid_data; //(7)
    FtoW->m_complementary_data = complementary_data; //(8)
    FtoW->m_PID_data = pid_data; //(9)
    FtoW->m_FtoW_data = FtoW_data; //(10)
    FtoW->m_xsens_state_data= xsens_data;
    maxon1->m_FtoW_data = FtoW_data; //(11)
    maxon2->m_FtoW_data = FtoW_data; //(12)
    maxon4->m_FtoW_data = FtoW_data; //(13)
    
    Serial * m_serial = new Serial("/dev/xsense", 9600);
    Xsens * m_xsens = new Xsens(); //initialise class Xsens
    Sensor * de_sensor = new Sensor(m_xsens,m_serial);

    m_xsens->m_xsens_state_data = xsens_data; //(0)
    de_sensor->m_xsens_state_data = xsens_data;
    de_sensor->m_ruwe_state_data = ruwe_data;  // (1)
  int counter = 4;
    while (true) {
        high_resolution_clock::time_point t1= high_resolution_clock::now();
        de_sensor->get_data();
        if (counter ==0){
        filter->FilterIt();
        com_filter->CalculateRealHeight();            
        PID->PID_in();
        FtoW->MMA();
       
        maxon1->Move();
        maxon2->Move();
        this_thread::sleep_for(chrono::milliseconds(5));
        maxon4->Move();
        counter=5;
        }
        counter--;
    high_resolution_clock::time_point t2= high_resolution_clock::now();
    ms d = std::chrono::duration_cast<ms>(t1-t2);
    int t_total = 12500-d.count();
    printf("loop tijd is %i",t_total);
    this_thread::sleep_for(chrono::microseconds(t_total));
    
    }
    // Als de klus klaar is ruim de objecten op.    
    delete (xsens_data);
    delete (ruwe_data);
    delete (filtered_data);
    delete (pid_data);
    delete (de_sensor);
    delete (filter);
    delete (complementary_data);
    delete (PID);
    delete (com_filter);
    delete (FtoW);
    delete (maxon1);
    delete (maxon2);
    delete (maxon4);

    return 0;
}