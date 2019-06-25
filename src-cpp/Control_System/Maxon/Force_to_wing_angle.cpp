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



ForceToWingAngle::ForceToWingAngle(DataStore * const control_data, PID_caller * const pid_caller) 
  : control_data_(control_data), pid_caller_(pid_caller) {
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
  DataStore::RealData real_data =control_data_-> GetRealData();
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
  
  //First calculate the required force the wing should supply based on the pitch
  // roll and height force and the inverse MMA
  float left_force = 0;
  float right_force = 0;
  float back_force = 0;
  
  // Once the minimal roll speed has been reachted the force should be computed for the roll
  if (velocity_>kMinSpeedRoll){    //snelheid hoger dan 2m/s
    pid_caller_->compute_pid_roll();
     //input.Force_height = 0;
    left_force += compute_force_(0, pid_data.Force_pitch, pid_data.Force_roll, inverse_matrix_MMA_[0]);
    right_force += compute_force_(0, pid_data.Force_pitch, pid_data.Force_roll, inverse_matrix_MMA_[1]);
    back_force += compute_force_(0, pid_data.Force_pitch, pid_data.Force_roll, inverse_matrix_MMA_[2]);
  }
  
  // Once the kMinSpeedHeight has been reach the boat shall start trying to get into a certain angle;
  if (velocity_>kMinSpeedHeight){
    pid_caller_->compute_pid_height();
    //When the height is larger than the mimimum height e.g. the boat is flying the Force Height should be added to the
    // Force
    if(real_data.Real_height>kMinHeight){
     left_force += compute_force_(pid_data.Force_height, 0, 0, inverse_matrix_MMA_[0]);
     right_force += compute_force_(pid_data.Force_height, 0, 0, inverse_matrix_MMA_[1]);
     back_force += compute_force_(pid_data.Force_height, 0, 0, inverse_matrix_MMA_[2]);    
    } else {
      //If the boat is not yet flying a constant force is applied to get it into the air.
      left_force += kLiftOfForce;
      right_force += kLiftOfForce;
    }
  }
  
  //Next calculate the required lift coefficients
  float left_lift_coefficient;
  float right_lift_coefficient;
  float back_lift_coefficient;
  
  //Only calculate it when the velocity is not equal to 0 else we get a divided by 0 error
  if(velocity_!=0){
     //devide the force by (0.5*density*surface*velocity) to get the lift coefficient 
    left_lift_coefficient = compute_lift_coefficient_(left_force, kLeftSurface);
    right_lift_coefficient = compute_lift_coefficient_(right_force, kRightSurface);
    back_lift_coefficient = compute_lift_coefficient_(back_force, kBackSurface);   
  } else {
    left_lift_coefficient = 0; 
    right_lift_coefficient = 0;
    back_lift_coefficient = 0;
  }
  
  //Next calculate the required angles compared to the water and 0 lift angle
  float left_angle_total = compute_wing_angle_(left_lift_coefficient);
  float right_angle_total = compute_wing_angle_(right_lift_coefficient);
  float back_angle_total = compute_wing_angle_(back_lift_coefficient);

  //calculate the final wing angles correcting for the pitch and 0 lift angle
  float boat_pitch = real_data.Real_pitch;

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
}

//Compute the lift coefficient using required force, velocity and area.
float ForceToWingAngle::compute_lift_coefficient_(float force, float area) {
  float lift_coefficient =  force /(0.5 * kDensity * area * pow(velocity_, 2));
  return lift_coefficient;
}

float ForceToWingAngle::compute_force_(float force_height, float force_pitch, float force_roll, float row_inverse_matrix[]) {
  
  float force = row_inverse_matrix[0] * force_height + row_inverse_matrix[1] * force_pitch + row_inverse_matrix[2] * force_roll;

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

ForceToWingAngle* ForceToWingAngle::set_heigh_pid_start(float height) {
  kMinHeight = height;
  M_OK<<"SETTING HEIGHT PID START POINT TO: "<<height;
}

ForceToWingAngle* ForceToWingAngle::set_lift_of_force(int force) {
  kLiftOfForce = force;
  M_OK<<"SETTING LIFT OF FORCE TO: "<<force;

}

ForceToWingAngle* ForceToWingAngle::set_lift_start_velocity(float speed) {
  kMinSpeedHeight = speed;
  M_OK<<"SETTING LIFT START VELOCITY TO: "<<speed;
}

ForceToWingAngle* ForceToWingAngle::set_roll_start_velocity(float velocity) {
  kMinSpeedRoll = velocity;
  M_OK<<"SETTING ROLL START VELOCITY TO: "<<velocity;
}




