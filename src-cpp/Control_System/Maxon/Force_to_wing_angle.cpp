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



ForceToWingAngle::ForceToWingAngle(DataStore * const control_data, PID_caller * const pid_caller, Vlotter * const vlotter) 
  : control_data_(control_data), pid_caller_(pid_caller), vlotter_(vlotter), config(new ConfigReader()) {
  calculate_inverse_matrix();
}

ForceToWingAngle::~ForceToWingAngle() {
  delete config;
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
  DataStore::PIDDataTotal pid_data = control_data_-> GetPIDData();
  DataStore::RealData real_data = control_data_-> GetRealData();
  DataStore::XsensData xsense_data = control_data_->GetXsensData();
  DataStore::PIDDataSplit split_PID_data = control_data_->GetPIDSplitData();

  DataStore::AngleWings output_wing_angles;
   
  DataStore::EposData epos_data = control_data_->GetEposData();
  
  epos_data.fly_mode_state = 1;
//  
//  if (kUseXsensVelocity){
//    set_velocity(xsense_data.velocity_magnitude);
//  } else {
//    set_velocity(power_input->driver.motor_speed / kRpmToMeterPerSecond);
//  }
//     
//  if (kControlTesting){
//    set_velocity(kControlTestVelocity);
//  }
//  M_INFO<<"Acceleration x: "<<xsense_data.acceleration_x << " | Speed x: "<<velocity_;
//  
//  //First calculate the required force the wing should supply based on the pitch
//  // roll and height force and the inverse MMA
//  float left_force = 0;
//  float right_force = 0;
//  float back_force = 0;
//  
//  // Once the minimal roll speed has been reachted the force should be computed for the roll
//  if (velocity_>kMinSpeedRoll){    //snelheid hoger dan 2m/s
//    epos_data.fly_mode_state = 2;
//    pid_caller_->compute_pid_roll();
//     //input.Force_height = 0;
//    left_force += compute_force_(0, pid_data.Force_pitch, pid_data.Force_roll, inverse_matrix_MMA_[0]);
//    right_force += compute_force_(0, pid_data.Force_pitch, pid_data.Force_roll, inverse_matrix_MMA_[1]);
//    back_force += compute_force_(0, pid_data.Force_pitch, pid_data.Force_roll, inverse_matrix_MMA_[2]);
//  } else {
//    pid_caller_->reset_integral_roll();
//  }
//  
//  if(real_data.Real_roll <0.01 && real_data.Real_roll>-0.01){
//    pid_caller_->reset_integral_roll();
//  }
//
//  // Once the kMinSpeedHeight has been reach the boat shall start trying to get into a certain angle;
//  if (velocity_>kMinSpeedHeight){
//    epos_data.fly_mode_state = 3;
//    pid_caller_->compute_pid_height();
//    split_PID_data.P_roll = split_PID_data.P_roll/10;
//    split_PID_data.D_roll = split_PID_data.D_roll/10;
//    //When the height is larger than the mimimum height e.g. the boat is flying the Force Height should be added to the
//    // Force
//    M_OK<<"MINIMUM SPEED REACHED";
//    if(real_data.Real_height>kMinHeight){
//     left_force += compute_force_(pid_data.Force_height, 0, 0, inverse_matrix_MMA_[0]);
//     right_force += compute_force_(pid_data.Force_height, 0, 0, inverse_matrix_MMA_[1]);
//     back_force += compute_force_(pid_data.Force_height, 0, 0, inverse_matrix_MMA_[2]);  
//     epos_data.fly_mode_state = 4;
//    }    
//    else {
//      //If the boat is not yet flying a constant force is applied to get it into the air.
//      left_force += kLiftOfForce;
//      right_force += kLiftOfForce;
//      M_OK<<"TRYING TO MAKE THE BOAT FLY";
//    }
//  } else {
//    pid_caller_->reset_integral_height();
//  }
//    epos_data.left_force = left_force;
//    epos_data.right_force = right_force;
//    epos_data.back_force = back_force;
//    control_data_->PutEposData(&epos_data);
//
//  //Next calculate the required lift coefficients
//  float left_lift_coefficient;
//  float right_lift_coefficient;
//  float back_lift_coefficient;
//  
//  //Only calculate it when the velocity is not equal to 0 else we get a divided by 0 error
//  if(velocity_>0){
//     //devide the force by (0.5*density*surface*velocity) to get the lift coefficient 
//    left_lift_coefficient = compute_lift_coefficient_(left_force, kLeftSurface);
//    right_lift_coefficient = compute_lift_coefficient_(right_force, kRightSurface);
//    back_lift_coefficient = compute_lift_coefficient_(back_force, kBackSurface);   
//  } else {
//    left_lift_coefficient = 0; 
//    right_lift_coefficient = 0;
//    back_lift_coefficient = 0;
//  }
//  
//  //Next calculate the required angles compared to the water and 0 lift angle
//  float left_angle_total = compute_wing_angle_(left_lift_coefficient);
//  float right_angle_total = compute_wing_angle_(right_lift_coefficient);
//  float back_angle_total = compute_wing_angle_(back_lift_coefficient);

  //calculate the final wing angles correcting for the pitch and 0 lift angle
  //float boat_pitch = real_data.Real_pitch;
  float boat_pitch = real_data.Real_pitch;
  float height_left = vlotter_->get_height(control::EncoderNumber::ENCODER_LEFT);
  float height_right = vlotter_->get_height(control::EncoderNumber::ENCODER_RIGHT);
//  output_wing_angles.Wing_left = compute_real_angle_(left_angle_total,boat_pitch, kZeroLiftAngle);
//  output_wing_angles.Wing_right = compute_real_angle_(right_angle_total, boat_pitch, kZeroLiftAngle);
  std::vector<float> function_values = config->get_function_parameters();
  if (function_values.size() == 4) {
    split_PID_data.a_left = function_values[0];
    split_PID_data.b_left = function_values[1];
    split_PID_data.a_right = function_values[2];
    split_PID_data.b_right = function_values[3];
  }
  split_PID_data.D_height = 0;
  split_PID_data.I_height = 0;
  split_PID_data.P_height = 0;
  split_PID_data.P_roll = 0;
  split_PID_data.I_roll = 0;
  split_PID_data.D_roll = 0;
  
  std::cout<<split_PID_data.a_left<<" "<<split_PID_data.b_left<<" "<<split_PID_data.a_right<<" "<<split_PID_data.b_right<<" "<<"\n";
  
  control_data_->PutPIDSplitData(&split_PID_data);
//  output_wing_angles.Wing_left = -0.65*height_left + 0.3; //standard is 0.17 -0.44
//  output_wing_angles.Wing_right = -0.65*height_right + 0.3; //standard is 0.17
  output_wing_angles.Wing_left = split_PID_data.a_left*height_left + split_PID_data.b_left; //standard is 0.17 -0.44
  output_wing_angles.Wing_right = split_PID_data.a_right*height_right + split_PID_data.b_right; //standard is 0.17
   
  output_wing_angles.Wing_back  = 0;//compute_real_angle_(back_angle_total, boat_pitch, kZeroLiftAngle);

  //Autoput the resulting data neatly
  M_INFO << "acceleration x: "<<xsense_data.acceleration_x << " | snelheid x: "<< velocity_ 
//         << "\nlinks Cl: "<<left_lift_coefficient << " | rechts Cl: "<<right_lift_coefficient
//         << "\nAchter Cl: "<<back_lift_coefficient << " | lift hoogte: "<<pid_data.Force_height
//         << "\nlift roll: "<<pid_data.Force_roll << " | lift pitch: "<<pid_data.Force_pitch
//         << "\nlift links: "<<left_force << " | lift rechts: "<<right_force
//         << "\nlift achter: "<<back_force << " | vleugelhoek links: "<<output_wing_angles.Wing_left
         << "\nvleugelhoek rechts: "<<output_wing_angles.Wing_right << " | vleugelhoek achter: "<<output_wing_angles.Wing_back;

  control_data_->PutWingData(&output_wing_angles);
}

//Compute the lift coefficient using required force, velocity and area.
float ForceToWingAngle::compute_lift_coefficient_(float force, float area) {
  float lift_coefficient =  force /(0.5 * kDensity * area * pow(velocity_, kVelocityMultiplier));
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
  float water_angle = original_angle + zero_lift_angle;
    if (water_angle>kStallAngle){
    water_angle = kStallAngle;
  } 
  if (water_angle<-kStallAngle) {
    water_angle = -kStallAngle;
  }
  float real_angle = water_angle-pitch_correction;
  return real_angle;
}



void ForceToWingAngle::set_velocity(float velocity) {
  velocity_ = velocity;
}

ForceToWingAngle* ForceToWingAngle::set_heigh_pid_start(float height) {
  kMinHeight = height-0.001;
  M_OK<<"SETTING HEIGHT PID START POINT TO: "<<kMinHeight;
  return this;
}

ForceToWingAngle* ForceToWingAngle::set_lift_of_force(int force) {
  kLiftOfForce = force;
  M_OK<<"SETTING LIFT OF FORCE TO: "<<force;
  return this;
}

ForceToWingAngle* ForceToWingAngle::set_lift_start_velocity(float speed) {
  kMinSpeedHeight = speed-0.001;
  M_OK<<"SETTING LIFT START VELOCITY TO: "<<kMinSpeedHeight;
  return this;
}

ForceToWingAngle* ForceToWingAngle::set_roll_start_velocity(float velocity) {
  kMinSpeedRoll = velocity-0.001;
  M_OK<<"SETTING ROLL START VELOCITY TO: "<<kMinSpeedRoll;
  return this;
}

ForceToWingAngle* ForceToWingAngle::set_zero_lift(float zero_lift_angle) {
  kZeroLiftAngle = zero_lift_angle;
  M_OK<<"SETTING ZERO LIFT ANGLE TO: "<<kZeroLiftAngle;
  return this;
  
}



