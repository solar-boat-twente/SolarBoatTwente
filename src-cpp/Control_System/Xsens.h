/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Xsens.h
 * Author: Danielle Kruijver
 *
 * Created on 23 april 2019, 16:57
 */
#ifndef XSENS_H
#define XSENS_H
#include "../../lib-cpp/Serial/Serial.h"
//#include "structures.h"
#include "DataStore.h"

namespace MIO{
namespace xsens{

enum XsensMessage{
  Xsens_PREAMBLE,
  Xsens_BID,
  Xsens_MID,
  Xsens_LENGTH,
  Xsens_DATA,
  Xsens_CHECKSUM  
};

enum XsensStates{
  ID,
  Euler_Angle,
  Acceleration,
  LatLon,
  Velocity
};

struct XsensParser{
  enum XsensMessage Message;
  int DATA_counter;
  int DATA_length;
  char DATA[256];
  char CHECKSUM;
  bool Succesfull_received;
  };

//struct XsensData{
//  float roll;
//  float yaw;
//  float pitch;
//  float acceleration_x;
//  float acceleration_y;
//  float acceleration_z;
//  float velocity_x;
//  float velocity_y;
//  float velocity_z;
//  float latitude;
//  float longitude;            
//};

struct Array_Output{
  float arr_roll[10];
  float arr_yaw[10];
  float arr_pitch[10];
  float arr_acceleration_x[10];
  float arr_acceleration_y[10];
  float arr_acceleration_z[10];
  float arr_velocity_x[10];
  float arr_velocity_y[10];
  float arr_velocity_z[10];
  float arr_latitude[10];
  float arr_longitude[10];  
};



class Xsens {
public:
  Xsens();
  Xsens(const Xsens& orig);
  
  DataStore *m_xsens_state_data;
  virtual ~Xsens();
  
  /**
   * This function parses the message in the different parts of the struct Xsensparser
   * @param Parser this is a pointer that points to the structure XsensParser
   * @param byte the message is separated on the basis of bytes
   */
  void ParseMessage(uint8_t byte[]);
  
  /**
   * In this function the data is sorted in such a way that the different data types are sorted
   * @param data this is the data that comes from the xsens and has to be sorted
   * @return this returns the sorted data
   */
  void ParseData();
  
private:

  /**
   * This function is used to write the values from the xsens as floats instead as bytes
   * @param loc location of the array of characters
   * @return float values
   */
  float pointer2float(char *loc);
  
  XsensParser * Parser = new XsensParser;
};
}
}

#endif /* XSENS_H */