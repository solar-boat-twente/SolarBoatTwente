/* 
 * File:   Force_to_wing_angle.cpp
 * Author: User
 * 
 * Created on 28 februari 2019, 10:05
 */

#include <iostream>
#include <cmath>
#include <math.h>

#include "../../../lib-cpp/Debugging/easy_debugging.hpp"

#include "Force_to_wing_angle.h"

using namespace std;
using namespace MIO;
using namespace Control;



ForceToWingAngle::ForceToWingAngle(){
  calculate_inverse_matrix();
}

ForceToWingAngle::ForceToWingAngle(const ForceToWingAngle& orig) {
}

ForceToWingAngle::~ForceToWingAngle() { }

void ForceToWingAngle::calculate_inverse_matrix() {
   // TODO: replace magic numbers!
  float matrix_MMA[3][3] = {
        {1, 1, 1},
        {1.54, 1.54, -2.83},
        {-0.37, 0.37, 0},
  }; 
    
  float determinant = 0;

  // finding determinant:
  for(int i = 0; i < 3; i++){
          determinant = determinant + (matrix_MMA[0][i] * (matrix_MMA[1][(i+1)%3] * matrix_MMA[2][(i+2)%3] - matrix_MMA[1][(i+2)%3] * matrix_MMA[2][(i+1)%3]));
  }
   inverse_matrix_MMA_[0][0] = ((matrix_MMA[(0+1)%3][(0+1)%3] * matrix_MMA[(0+2)%3][(0+2)%3]) - (matrix_MMA[(0+1)%3][(0+2)%3] * matrix_MMA[(0+2)%3][(0+1)%3]))/ determinant;
   inverse_matrix_MMA_[0][1] = ((matrix_MMA[(1+1)%3][(0+1)%3] * matrix_MMA[(1+2)%3][(0+2)%3]) - (matrix_MMA[(1+1)%3][(0+2)%3] * matrix_MMA[(1+2)%3][(0+1)%3]))/ determinant;
   inverse_matrix_MMA_[0][2] = ((matrix_MMA[(2+1)%3][(0+1)%3] * matrix_MMA[(2+2)%3][(0+2)%3]) - (matrix_MMA[(2+1)%3][(0+2)%3] * matrix_MMA[(2+2)%3][(0+1)%3]))/ determinant;   
   inverse_matrix_MMA_[1][0] = ((matrix_MMA[(0+1)%3][(1+1)%3] * matrix_MMA[(0+2)%3][(1+2)%3]) - (matrix_MMA[(0+1)%3][(1+2)%3] * matrix_MMA[(0+2)%3][(1+1)%3]))/ determinant;
   inverse_matrix_MMA_[1][1] = ((matrix_MMA[(1+1)%3][(1+1)%3] * matrix_MMA[(1+2)%3][(1+2)%3]) - (matrix_MMA[(1+1)%3][(1+2)%3] * matrix_MMA[(1+2)%3][(1+1)%3]))/ determinant;
   inverse_matrix_MMA_[1][2] = ((matrix_MMA[(2+1)%3][(1+1)%3] * matrix_MMA[(2+2)%3][(1+2)%3]) - (matrix_MMA[(2+1)%3][(1+2)%3] * matrix_MMA[(2+2)%3][(1+1)%3]))/ determinant;
   inverse_matrix_MMA_[2][0] = ((matrix_MMA[(0+1)%3][(2+1)%3] * matrix_MMA[(0+2)%3][(2+2)%3]) - (matrix_MMA[(0+1)%3][(2+2)%3] * matrix_MMA[(0+2)%3][(2+1)%3]))/ determinant;
   inverse_matrix_MMA_[2][1] = ((matrix_MMA[(1+1)%3][(2+1)%3] * matrix_MMA[(1+2)%3][(2+2)%3]) - (matrix_MMA[(1+1)%3][(2+2)%3] * matrix_MMA[(1+2)%3][(2+1)%3]))/ determinant;
   inverse_matrix_MMA_[2][2] = ((matrix_MMA[(2+1)%3][(2+1)%3] * matrix_MMA[(2+2)%3][(2+2)%3]) - (matrix_MMA[(2+1)%3][(2+2)%3] * matrix_MMA[(2+2)%3][(2+1)%3]))/ determinant;
   
   std::cout << (inverse_matrix_MMA_[1][3]) << "\t";
   std::cout << (inverse_matrix_MMA_[2][3]) << "\t";

}


void ForceToWingAngle::MMA(structures::PowerInput * power_input) {

    /* First we will start with the MMA. This is the motor mixing algorithm that 
     * divides the incoming forces from the 3 PID controllers in 3 different 
     * forces that the wings should deliver. The MMA consists of a matrix, that 
     * has to be inversed to calculate the right forces per wing. 
    */
   DataStore::PIDDataTotal input =PID_data_-> GetPIDData();
   DataStore::RealData input2 =complementary_data_-> GetComplementaryData();
   DataStore::AngleWings output;
   DataStore::XsensData v = xsens_state_data_->GetXsensData();
   
   //velocity_ = power_input->driver.motor_speed/216;
//   velocity_ = v.velocity_x;
   //velocity_ += v.acceleration_x *0.0125;
   velocity_ = 4;
   M_INFO<<"Acceleration x: "<<v.acceleration_x << " | Speed x: "<<velocity_;
   
   if (velocity_ > MIN_SPEED) {    //snelheid hoger dan 2m/s
    
     // TODO: replace with function like `float compute_force(...)`
     float left_force = inverse_matrix_MMA_[0][0] * input.Force_height + inverse_matrix_MMA_[0][1] * input.Force_pitch + inverse_matrix_MMA_[0][2] * input.Force_roll;
     float right_force = inverse_matrix_MMA_[1][0] * input.Force_height + inverse_matrix_MMA_[1][1] * input.Force_pitch + inverse_matrix_MMA_[1][2] * input.Force_roll;
     float back_force = inverse_matrix_MMA_[2][0] * input.Force_height + inverse_matrix_MMA_[2][1] * input.Force_pitch + inverse_matrix_MMA_[2][2] * input.Force_roll;
     
     //devide the force by (0.5*density*surface*velocity) to get the lift coefficient 
     // TODO: replace with function like `float compute_lift_coefficient(float area)`
     float left_lift_coefficient = left_force / (0.5 * kDensity * kLeftSurface * pow(velocity_,2)) ; 
     float right_lift_coefficient = right_force / (0.5 * kDensity * kRightSurface  *pow(velocity_,2)) ;
     float back_lift_coefficient = back_force / (0.5 * kDensity * kBackSurface * pow(velocity_,2)) ;
    
     float left_angle_total = left_lift_coefficient / kLiftSlope;
     float right_angle_total = right_lift_coefficient / kLiftSlope;
     float back_angle_total = back_lift_coefficient / kLiftSlope;
      
     //calculate the final wing angles*
     output.Wing_left = left_angle_total + input2.Real_pitch - kZeroLiftAngle;
     output.Wing_right = right_angle_total + input2.Real_pitch - kZeroLiftAngle;
     output.Wing_back  = back_angle_total + input2.Real_pitch - kZeroLiftAngle;
     
     //Autoput the resulting data neatly
     M_INFO << "acceleration x: "<<v.acceleration_x << " | snelheid x: "<< velocity_ 
            << "\nlinks Cl: "<<left_lift_coefficient << " | rechts Cl: "<<right_lift_coefficient
            << "\nAchter Cl: "<<back_lift_coefficient << " | lift hoogte: "<<input.Force_height
            << "\nlift roll: "<<input.Force_roll << " | lift pitch: "<<input.Force_pitch
            << "\nlift links: "<<left_force << " | lift rechts: "<<right_force
            << "\nlift achter: "<<back_force << " | vleugelhoek links: "<<output.Wing_left
            << "\nvleugelhoek rechts: "<<output.Wing_right << " | vleugelhoek achter: "<<output.Wing_back;
     
     FtoW_data_->PutWingData(&output);
     /*Now we can calculate the angle that the maxon motor should make in such 
      * a way that the wing is making the right angle.   
      */
   } else {
    output.Wing_left = -0.07;//left_angle_total - input2.Real_pitch - kZeroLiftAngle;
    output.Wing_right = -0.07;//right_angle_total - input2.Real_pitch - kZeroLiftAngle;
    output.Wing_back = -0.07;//back_angle_total - input2.Real_pitch - kZeroLiftAngle;
    cout << "Else statement bereikt"  << "\r\n";
    cout << "Wing_left in force to wing angle" << output.Wing_left << "\r\n";
    FtoW_data_->PutWingData(&output);
   }
}