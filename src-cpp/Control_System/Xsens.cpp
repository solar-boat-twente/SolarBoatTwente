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
#include "DataStore.h"
#include "Xsens.h"

using namespace MIO;

xsens::Xsens::Xsens() {
}

xsens::Xsens::Xsens(const Xsens& orig) {
}

xsens::Xsens::~Xsens() {
}

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
void xsens::Xsens::ParseMessage(uint8_t byte[]) {
  //xsens::XsensParser enumerator;
  //xsens::XsensParser * pointer_enum = &enumerator;
  //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  Parser->Message = Xsens_PREAMBLE;
  //printf("testjeee\r\n");
  int d=0;
  while (d<6){
  switch(Parser->Message){
    case Xsens_PREAMBLE:
      //printf("tstje2");
      if (byte[0]==0xFA){
        Parser->CHECKSUM = 0x00; //Preamble doet niet mee met de checksum vandaar 0x00
        Parser->Message = Xsens_BID;
        printf("PREAMBLE\r\n");
        d++;
      }
      //d = 6;
      break;
    
    case Xsens_BID:
      if (byte[1]==0xFF){
        Parser->CHECKSUM += byte[1]; //iedere keer de byte bij de checksum optellen
        Parser->Message = Xsens_MID;
        ///printf("BID\r\n");
        d++;
      }
      else{
        Parser->Succesfull_received = 0;
        Parser->Message = Xsens_PREAMBLE;
        d=0;
      }
      break;
      
    case Xsens_MID:
      if (byte[2]==0x36){
        Parser->CHECKSUM += byte[2];
        Parser->Message = Xsens_LENGTH;
       // printf("MID\r\n");
        d++;
      }
      else{
        Parser->Succesfull_received = 0;
        Parser->Message = Xsens_PREAMBLE;
        d=0;
      }
      break;
      
    case Xsens_LENGTH:
      Parser->CHECKSUM += byte[3];
      Parser->DATA_length = (int)byte[3]; //lengte is gelijk aan de waarde in de byte (hier omgezet naar een integer)
      Parser->Message = Xsens_DATA; //volgende stap is naar de data
      Parser->DATA_counter = 0; //voordat we naar de data gaan ook de counter op 0 zetten zodat je straks de lengte met de counter kan vergelijken
      //printf("LENGTH\r\n");
      d++;
      break;
      
    case Xsens_DATA:
      Parser->DATA_counter = 0;
      while (Parser->DATA_counter < Parser->DATA_length){
        Parser->CHECKSUM += byte[Parser->DATA_counter + 4];
        Parser->DATA[Parser->DATA_counter] = byte[Parser->DATA_counter + 4];
        Parser->DATA_counter++;
      };
       
      if (Parser->DATA_length == Parser->DATA_counter){
        Parser->Message = Xsens_CHECKSUM;
      }
      //printf("DATA\r\n");
      d++;
      break;
      
    case Xsens_CHECKSUM:
      Parser->CHECKSUM += byte[Parser->DATA_counter + 4 ];
      //printf("CHECKSUM: %i", Parser->CHECKSUM);
      if (Parser->CHECKSUM == 0x00){
        Parser->Succesfull_received = 1;
        //printf("CHECKSUM succes");
      }
      d++;
      break;
      
    default:
      Parser->Message = Xsens_PREAMBLE;
      d=0;
  }   
 }
}

void xsens::Xsens::ParseData(){
//  xsens::XsensData Xsens_data;
//  xsens::XsensData * DataStruct = &Xsens_data;
  DataStore::XsensData xsensor;  
  if (Parser->Succesfull_received = 1){
  int i = 0;
  XsensStates State = ID;
  while (i<Parser->DATA_length){
    char byte = Parser->DATA[i];
    char byte2 = Parser->DATA[i+1];
    //printf("lengte data: %i,%i\r\n", Parser->DATA_length,i);
    
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
        
        xsensor.roll = pointer2float(Parser->DATA + i + 2);
        xsensor.pitch = pointer2float(Parser->DATA + i + 6); 
        xsensor.yaw = pointer2float(Parser->DATA + i + 10); 
        std::cout << "xsens roll is\r\n "<<xsensor.roll ;
        std::cout << "xsens pitch is\r\n "<<xsensor.pitch ;
        //printf("pitch: %f\r\n", DataStruct->pitch);
        //printf("yaw: %f\r\n", DataStruct->yaw);
        i = i + 13;
        //std::cout << "i is "<<i ;
        m_xsens_state_data->PutXsensData(&xsensor);
        State = ID;
        break;
        
      case Acceleration:
        
        xsensor.acceleration_x = pointer2float(Parser->DATA + i + 2);
        xsensor.acceleration_y = pointer2float(Parser->DATA + i + 6);
        xsensor.acceleration_z = pointer2float(Parser->DATA + i + 10);
        //printf("acc_x: %f\r\n", DataStruct->acceleration_x);
        //printf("acc_y: %f\r\n", DataStruct->acceleration_y);
        //printf("acc_z: %f\r\n", DataStruct->acceleration_z);
        
        i = i + 13;
        State = ID;
        m_xsens_state_data->PutXsensData(&xsensor);
        break;
        
      case Velocity:
        xsensor.velocity_x = pointer2float(Parser->DATA + i + 2);
        xsensor.velocity_y = pointer2float(Parser->DATA + i + 6);
        xsensor.velocity_z = pointer2float(Parser->DATA + i + 10);
        //printf("vel_x: %f\r\n", DataStruct->velocity_x);
        //printf("vel_y: %f\r\n", DataStruct->velocity_y);
        //printf("vel_z: %f\r\n", DataStruct->velocity_z);
        State = ID;
        i = i + 13;
        m_xsens_state_data->PutXsensData(&xsensor);
        break;
        
      case LatLon:
        
        xsensor.latitude = pointer2float(Parser->DATA + i + 2);
        xsensor.longitude = pointer2float(Parser->DATA + i + 6);
        //printf("lat: %f\r\n", DataStruct->latitude);
        //printf("lon: %f\r\n", DataStruct->longitude);
        State = ID;
        i = i + 9;
        m_xsens_state_data->PutXsensData(&xsensor);
        break;
        
      default:
        State = ID;
        break;
    }
    //m_xsens_state_data->PutXsensData(&xsensor);
    i++;
  }
}
  else{
    //printf("Error in message xsens");
  }
}


