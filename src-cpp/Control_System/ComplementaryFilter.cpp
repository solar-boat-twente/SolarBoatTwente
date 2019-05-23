/* 
 * File:   ComplementaryFilter.cpp
 * Author: arjan
 * 
 * Created on 1 maart 2019, 11:49
 */
#include <stdio.h>      /* printf */
#include <math.h>       /* sin */
#include "ComplementaryFilter.h"

// in 2 angles from the sensors, roll from xsens, acceleration from xsens and out 1 roll , 1 hoogte 
ComplementaryFilter::ComplementaryFilter(){
   
}

ComplementaryFilter::ComplementaryFilter(const ComplementaryFilter& orig) {
}

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
  
   DataStore::FilteredData input =m_filtered_data-> GetFilteredData();
   DataStore::RealData realData;
   
   height_left= sin(input.filtered_angle_left)*Length_vlotter;
   height_right= sin(input.filtered_angle_right)*Length_vlotter;
   roll_front=asin((height_left-height_right)*distance_between_vlotters);
   height_xsens=input.filtered_Z_accel*pow(dt,2);
   //realData.Real_roll=(roll_front+input.filtered_roll)/2;
   realData.Real_roll=input.filtered_roll;
   realData.Real_height=(height_left+height_right+height_xsens)/3;
   realData.Real_pitch=input.filtered_pitch;
   printf("roll_front is %f\r\n",roll_front);
   printf("real angles zijn: %f,%f,%f\r\n",realData.Real_height,realData.Real_roll,realData.Real_pitch);  
   m_complementary_data->PutComplementaryData(&realData);
}           