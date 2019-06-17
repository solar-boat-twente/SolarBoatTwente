/* 
 * File:   Force_to_wing_angle.h
 * Author: User
 *
 * Created on 28 februari 2019, 10:05
 */

#ifndef FORCE_TO_WING_ANGLE_H
#define	FORCE_TO_WING_ANGLE_H

#include "../DataStore.h"
#include "../../../solarboattwente.h"

namespace MIO{
namespace Control{

const int MIN_SPEED = 2;

class ForceToWingAngle {
 public:

    ForceToWingAngle();
    ForceToWingAngle(const ForceToWingAngle& orig);
    virtual ~ForceToWingAngle();
    
    void add_data(DataStore * PID_data, DataStore * FtoW_data, 
        DataStore * complementary_data, DataStore * xsens_state_data){
      PID_data_ = PID_data;
      FtoW_data_ = FtoW_data;
      complementary_data_ = complementary_data;
      xsens_state_data_ = xsens_state_data;
    };
    
    void MMA(structures::PowerInput * power_input);    
 private:
  DataStore *PID_data_;
  DataStore *FtoW_data_;
  DataStore *complementary_data_;
  DataStore *xsens_state_data_;
  float inverse_matrix_MMA_[3][3];
  
  const float kLeftSurface = 0.5*(35832.62+9379.39+15937.11+43774.74) * 0.000006;   //m2
  const float kRightSurface = 0.5*(35832.62+9379.39+15937.11+43774.74) * 0.000006;       
  const float kBackSurface = 0.5*(37297+37930+10794+10794) * 0.000006;
  const float kDensity = 1025; //kg/m3 density salt water
  const int kLiftSlope = 5;
  const float kZeroLiftAngle = 0.08; //-0.05236;      //radians
    
  float velocity_ = 0;
  void calculate_inverse_matrix();
};   
}
}
#endif
