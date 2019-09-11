/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Xsens.cpp
 * Author: Danielle Kruijver
 * 
 * Created on 23 april 2019, 16:57
 */
#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <thread>
#include <chrono>
#include <iostream>
#include <numeric>
#include <cassert>
#include <complex>


#include "../../../lib-cpp/Debugging/easy_debugging.hpp"
#include "../DataStore.h"
#include "Xsens.h"

using namespace MIO;
using namespace std;

xsens::Xsens::Xsens(DataStore * const control_data) 
  : control_data_(control_data), parser_(new XsensParser) 
{}

xsens::Xsens::Xsens(const Xsens& orig) : control_data_(orig.control_data_), parser_(orig.parser_) {
  
}

xsens::Xsens::~Xsens() {
  delete(parser_);
}

// TODO: replace with *reinterpret_cast<float*>(loc), with correct endianness
float xsens::Xsens::pointer2float(char loc[]){
    union {
        char c[4];
        float f;
    } u;
    for (int i=0;i<4;i++){
        u.c[3-i]=loc[i];
    }
    return u.f;
}

/*------------------------------------------------------------------------------
CHECKSUM: This field is used for communication error-detection. If all message 
bytes excluding the preamble are summed and the lower byte value of the result 
equals zero, the message is valid and it may be processed. The checksum value 
of the message should be included in the summation. 
------------------------------------------------------------------------------*/
bool xsens::Xsens::parse_message(const uint8_t byte[]) {
  //xsens::XsensParser enumerator;
  //xsens::XsensParser * pointer_enum = &enumerator;
  //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  parser_->message = XsensMessage::Xsens_PREAMBLE;
  //printf("testjeee\r\n");
  int d=0;
  int byte_counter = 0;
  while (d<6){
    switch(parser_->message){
      case XsensMessage::Xsens_PREAMBLE:
        //printf("tstje2");
        M_INFO<<"PREAMBLE; byte counter: "<<byte_counter;

        if (byte[byte_counter]==0xFA) {
          parser_->CHECKSUM = 0x00; //Preamble doet niet mee met de checksum vandaar 0x00
          parser_->message = XsensMessage::Xsens_BID;
          d++;
        }
        //d = 6;
        break;

      case XsensMessage::Xsens_BID:
        if (byte[byte_counter]==0xFF) {
          parser_->CHECKSUM = byte[byte_counter]; //iedere keer de byte bij de checksum optellen
          parser_->message = XsensMessage::Xsens_MID;
          d++;
        } else {
          parser_->Succesfull_received = 0;
          parser_->message = XsensMessage::Xsens_PREAMBLE;
          d=0;
          }
        break;

      case XsensMessage::Xsens_MID:
        if (byte[byte_counter]==0x36) {
          parser_->CHECKSUM += byte[byte_counter];
          parser_->message = XsensMessage::Xsens_LENGTH;
         // printf("MID\r\n");
          d++;
        } else{
          parser_->Succesfull_received = 0;
          parser_->message = XsensMessage::Xsens_PREAMBLE;
          d=0;
        }
        break;

      case XsensMessage::Xsens_LENGTH:
        parser_->CHECKSUM += byte[byte_counter];
        parser_->DATA_length = (int)byte[byte_counter]; //lengte is gelijk aan de waarde in de byte (hier omgezet naar een integer)
        parser_->message = XsensMessage::Xsens_DATA; //volgende stap is naar de data
        parser_->DATA_counter = 0; //voordat we naar de data gaan ook de counter op 0 zetten zodat je straks de lengte met de counter kan vergelijken
        //printf("LENGTH\r\n");
        d++;
        break;

      case XsensMessage::Xsens_DATA:
        if (parser_->DATA_counter < parser_->DATA_length){
          parser_->CHECKSUM += byte[byte_counter];
          parser_->DATA[parser_->DATA_counter] = byte[byte_counter];
          parser_->DATA_counter++;
        };

        if (parser_->DATA_length == parser_->DATA_counter){
          parser_->message = XsensMessage::Xsens_CHECKSUM;
        }
        //printf("DATA\r\n");
        break;

      case XsensMessage::Xsens_CHECKSUM:
        parser_->CHECKSUM += byte[byte_counter];
        std::cout<<"CHECKSUM from Checksum: "<<parser_->CHECKSUM;
        if (parser_->CHECKSUM == 0x00){
          parser_->Succesfull_received = 1;
          d=6;
          M_OK<<"XSENS CHECKSUM SUCCESS";
          return true;
        } else {
          d = 0;
          parser_->message = XsensMessage::Xsens_PREAMBLE;
        }

        break;

      default:
        parser_->message = XsensMessage::Xsens_PREAMBLE;
        d=0;
        break;
    }
    //Now start looking at the next byte.
    if(byte_counter>150){
      M_WARN<<"READ NOTHING FROM XSENS";
      return false;
      break;
    }
    byte_counter++;
 }
}

void xsens::Xsens::parse_data(){
//  xsens::XsensData Xsens_data;
//  xsens::XsensData * DataStruct = &Xsens_data;
  DataStore::XsensData xsensor;  
  if (parser_->Succesfull_received = 1){
    int i = 0;
    XsensStates State = ID;

    while (i<parser_->DATA_length){
      //printf("lengte data: %i,%i\r\n", Parser->DATA_length,i);
      uint8_t byte = parser_->DATA[i];
      uint8_t byte2= parser_->DATA[i+1];
      switch (State){
        case ID:
            //std::cout<<"Testing ID"<<std::endl;
          if (byte==0x20 && byte2==0x30){
            State = Euler_Angle;
            }
          else if (byte==0x40 && byte2==0x20){
            //printf("aceleration case bereikt\r\n");
            State = Acceleration;
          }
          else if (byte==0xd0 && byte2==0x10){
            //printf("velocity case bereikt\r\n");
            State = Velocity;
          }
          else if (byte==0x50 && byte2==0x40){
            State = LatLon;
          }
          else{
            State = ID;
          }
          break;

        case Euler_Angle:
          //The angles of the xsens are all pointed exactly the opposite to how we defined them
          // Counter clockwise should be a positive roll!
          // Bow (Boeg) of the boat up is positive pitch!
          xsensor.roll = pointer2float(parser_->DATA + i + 2)*-1;
          xsensor.pitch = pointer2float(parser_->DATA + i + 6)*-1 + kPitchCorrection; 
          xsensor.yaw = pointer2float(parser_->DATA + i + 10); 
          M_INFO <<"xsens roll is "<<xsensor.roll;
          M_INFO << "xsens pitch is  "<<xsensor.pitch;
          i = i + 13;
          control_data_->PutXsensData(&xsensor);
          State = ID;
          break;

        case Acceleration:

          xsensor.acceleration_x = pointer2float(parser_->DATA + i + 2);
          xsensor.acceleration_y = pointer2float(parser_->DATA + i + 6);
          xsensor.acceleration_z = pointer2float(parser_->DATA + i + 10);


          i = i + 13;
          State = ID;
          control_data_->PutXsensData(&xsensor);
          break;

        case Velocity:
          xsensor.velocity_x = pointer2float(parser_->DATA + i + 2);
          xsensor.velocity_y = pointer2float(parser_->DATA + i + 6);
          xsensor.velocity_z = pointer2float(parser_->DATA + i + 10);
          xsensor.velocity_magnitude = sqrt(pow(xsensor.velocity_x,2)+pow(xsensor.velocity_y,2));
          //printf("vel_x: %f\r\n", DataStruct->velocity_x);
          //printf("vel_y: %f\r\n", DataStruct->velocity_y);
          //printf("vel_z: %f\r\n", DataStruct->velocity_z);
          State = ID;
          i = i + 13;
          //mtx.lock();
          control_data_->PutXsensData(&xsensor);
          //mtx.unlock();
          break;

        case LatLon:

          xsensor.latitude = pointer2float(parser_->DATA + i + 2);
          xsensor.longitude = pointer2float(parser_->DATA + i + 6);
          //printf("lat: %f\r\n", DataStruct->latitude);
          //printf("lon: %f\r\n", DataStruct->longitude);
          State = ID;
          i = i + 9;
          //mtx.lock();
          control_data_->PutXsensData(&xsensor);
          //mtx.unlock();
          break;

        default:
          State = ID;
          break;
      }
      //m_xsens_state_data->PutXsensData(&xsensor);
      i++;
    }
  } else{
    M_WARN<<"XSENS NOT SUCCESSFUL";
  }
}

