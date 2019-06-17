/* 
 * File:   ComplementaryFilter.cpp
 * Author: arjan
 * 
 * Created on 1 maart 2019, 11:49
 */
#include <stdio.h>      /* printf */
#include <math.h>       /* sin */

#include "../../../lib-cpp/Debugging/easy_debugging.hpp"

#include "ComplementaryFilter.h"

using namespace MIO;
using namespace Control;
// in 2 angles from the sensors, roll from xsens, acceleration from xsens and out 1 roll , 1 hoogte 


ComplementaryFilter::~ComplementaryFilter() {
}

void ComplementaryFilter::CalculateRealHeight()
{
  
   float Length_vlotter=0.7;
   float height_left=0;
   float height_right=0;
   float roll_front=0;
   float distance_between_vlotters=1.3;
   float height_xsens=0;
   float dt=0.0125;
  
   DataStore::FilteredData input = m_filtered_data->GetFilteredData();
   DataStore::RealData real_data;
   
   // Calculate the height of the left and right side of the boat
   height_left= sin(input.filtered_angle_left)*VLOTTER_LENGTH;
   height_right= sin(input.filtered_angle_right)*VLOTTER_LENGTH;
   
   // Calculate the roll from the difference in heights
   roll_front=asin((height_left-height_right)*distance_between_vlotters);
   
   // Calculate the height from the XSens data
   //height_xsens=input.filtered_Z_accel+input.filtered_Z_accel*pow(dt,2); //TODO: Hier klopt niks van....
   height_xsens = input.filtered_angle_right;
   //realData.Real_roll=(roll_front+input.filtered_roll)/2;
   real_data.Real_roll=input.filtered_roll;
   real_data.Real_height=height_xsens;//(height_left+height_right+height_xsens)/3; --> Dit is geen complementair filter.
   real_data.Real_pitch=input.filtered_pitch;
   M_INFO<<"Roll front equal to: "<< roll_front;
   M_INFO<<"Real height: "<<real_data.Real_height<<" Real Roll: "<<real_data.Real_roll
       <<" Real Pitch: "<<real_data.Real_pitch;
   m_complementary_data->PutComplementaryData(&real_data);
}           