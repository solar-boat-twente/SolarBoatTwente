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
using namespace control;



ForceToWingAngle::ForceToWingAngle(DataStore * const control_data) : control_data_(control_data) {
  calculate_inverse_matrix();
}

void ForceToWingAngle::calculate_inverse_matrix() {
  //Funtion that works for the 3*3 matrix but that is all
  //Matrix is defined inside the class    
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
   DataStore::PIDDataTotal pid_data =control_data_-> GetPIDData();
   DataStore::RealData complementary_data_input =control_data_-> GetComplementaryData();
  DataStore::XsensData xsense_data = control_data_->GetXsensData();

   DataStore::AngleWings output_wing_angles;
   
   
  if (kUseXsensVelocity){
    set_velocity(xsense_data.velocity_x);
  } else {
    set_velocity(power_input->driver.motor_speed / kRpmToMeterPerSecond);
  }
     
  if (kControlTesting){
     set_velocity(kControlTestVelocity);
  }
  M_INFO<<"Acceleration x: "<<xsense_data.acceleration_x << " | Speed x: "<<velocity_;

  if (velocity_ > kControlMinSpeed) {    //snelheid hoger dan 2m/s

   
    float left_force = compute_force_(pid_data, inverse_matrix_MMA_[0]);
    float right_force = compute_force_(pid_data, inverse_matrix_MMA_[1]);
    float back_force = compute_force_(pid_data, inverse_matrix_MMA_[2]);

     
    float left_lift_coefficient = compute_lift_coefficient_(left_force, kLeftSurface);
    float right_lift_coefficient = compute_lift_coefficient_(right_force, kRightSurface);
    float back_lift_coefficient = compute_lift_coefficient_(back_force, kBackSurface);

    float left_angle_total = compute_wing_angle_(left_lift_coefficient);
    float right_angle_total = compute_wing_angle_(right_lift_coefficient);
    float back_angle_total = compute_wing_angle_(back_lift_coefficient);

    //calculate the final wing angles*
    float boat_pitch = complementary_data_input.Real_pitch;
    
    
    output_wing_angles.Wing_left = compute_real_angle_(left_angle_total,boat_pitch);
    output_wing_angles.Wing_right = compute_real_angle_(right_angle_total, boat_pitch);
    output_wing_angles.Wing_back  = compute_real_angle_(back_angle_total, boat_pitch);

    //Autoput the resulting data neatly
    M_INFO << "acceleration x: "<<xsense_data.acceleration_x << " | snelheid x: "<< velocity_ 
           << "\nlinks Cl: "<<left_lift_coefficient << " | rechts Cl: "<<right_lift_coefficient
           << "\nAchter Cl: "<<back_lift_coefficient << " | lift hoogte: "<<pid_data.Force_height
           << "\nlift roll: "<<pid_data.Force_roll << " | lift pitch: "<<pid_data.Force_pitch
           << "\nlift links: "<<left_force << " | lift rechts: "<<right_force
           << "\nlift achter: "<<back_force << " | vleugelhoek links: "<<output_wing_angles.Wing_left
           << "\nvleugelhoek rechts: "<<output_wing_angles.Wing_right << " | vleugelhoek achter: "<<output_wing_angles.Wing_back;

    control_data_->PutWingData(&output_wing_angles);
    /*Now we can calculate the angle that the maxon motor should make in such 
     * a way that the wing is making the right angle.   
     */
  } else {
   output_wing_angles.Wing_left = -0.07;//left_angle_total - input2.Real_pitch - kZeroLiftAngle;
   output_wing_angles.Wing_right = -0.07;//right_angle_total - input2.Real_pitch - kZeroLiftAngle;
   output_wing_angles.Wing_back = -0.07;//back_angle_total - input2.Real_pitch - kZeroLiftAngle;
   cout << "Else statement bereikt"  << "\r\n";
   cout << "Wing_left in force to wing angle" << output_wing_angles.Wing_left << "\r\n";
   control_data_->PutWingData(&output_wing_angles);
  }
}

//Compute the lift coefficient using required force, velocity and area.
float ForceToWingAngle::compute_lift_coefficient_(float force, float area) {
  float lift_coefficient =  force /(0.5 * kDensity * area * pow(velocity_, 2));
  return lift_coefficient;
}

float ForceToWingAngle::compute_force_(DataStore::PIDDataTotal &pid_forces, float row_inverse_matrix[]) {
  
  float force = row_inverse_matrix[0] * pid_forces.Force_height + row_inverse_matrix[1] * pid_forces.Force_pitch + row_inverse_matrix[2] * pid_forces.Force_roll;

  return force;

}

float ForceToWingAngle::compute_wing_angle_(float lift_coefficient, int lift_to_slope) {
  float wing_angle = lift_coefficient/lift_to_slope;
  
  return wing_angle;
}

float ForceToWingAngle::compute_real_angle_(float original_angle, float pitch_correction, float zero_lift_angle) {
  //When the boat has a positve pitch the angles of the wings should be to the other side
  // The zero lift angle is negative the real angle should also be lower
  float real_angle = original_angle - pitch_correction + zero_lift_angle;

}



void ForceToWingAngle::set_velocity(float velocity) {
  velocity_ = velocity;
}


