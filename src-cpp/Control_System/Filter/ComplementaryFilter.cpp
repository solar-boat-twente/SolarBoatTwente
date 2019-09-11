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
using namespace control;
// in 2 angles from the sensors, roll from xsens, acceleration from xsens and out 1 roll , 1 hoogte 

ComplementaryFilter::ComplementaryFilter(DataStore * const control_data, Vlotter * const vlotter)
  : control_data_(control_data), vlotter_(vlotter) {

  };



void ComplementaryFilter::CalculateRealHeight()
{  
  
   DataStore::FilteredData input_from_filter = control_data_->GetFilteredData();
   DataStore::RealData real_data;
   
   // Calculate the height of the left and right side of the boat
   //float height_left = vlotter_->get_height(control::EncoderNumber::ENCODER_LEFT);
   //float height_right = vlotter_->get_height(control::EncoderNumber::ENCODER_RIGHT);
   
   // Calculate the roll from the difference in heights
   //float roll = vlotter_->get_roll_rad();
   
   real_data.Real_roll = input_from_filter.filtered_roll;
   real_data.Real_height = 0;// (height_right + height_left)/2;// --> Dit is geen complementair filter.
   real_data.Real_pitch = input_from_filter.filtered_pitch;
   
   //M_INFO<<"Roll front equal to: "<< roll;
   M_INFO<<"Real height: "<<real_data.Real_height<<" Real Roll: "<<real_data.Real_roll
       <<" Real Pitch: "<<real_data.Real_pitch;
   control_data_->PutRealData(&real_data);
}           