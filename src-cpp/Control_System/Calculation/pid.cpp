/* 
 * File:   pid.cpp
 * Author: User
 * 
 * Created on 27 februari 2019, 10:28
 */
#include <iostream>
#include <cmath>

#include "../../../lib-cpp/Debugging/easy_debugging.hpp"
#include "pid.h"

using namespace std;
using namespace MIO;
using namespace Control;
using namespace structures;
//float dt, float Kp, float Kd, float Ki, float Fc
void PID::set_PID_roll(structures::PIDState state){
  
//  k->PID_roll=STATE8;//STATE3;
  switch(state){
    case STATE1: 
      pimpl->update_values(0.0125,0,0,0); //laf
      M_INFO<<"Roll State 1 is used"; 
      break;
    case STATE2: 
      pimpl->update_values(0.0125,21.218,1.289,48.819);  //heel laf
      M_INFO<<"Roll State 2 is used"; 
      break;
    case STATE3: 
      pimpl->update_values(0.0125,414.957,-326.505,80.103);  // zeer aggressief
      M_INFO<<"Roll State 3 is used"; 
      break;
    case STATE4: 
      pimpl->update_values(0.01251,269.078,-169.860,48.055); 
      M_INFO<<"Roll State 4 is used"; 
      break;
    case STATE5: 
      pimpl->update_values(0.0125,79.228,5,0); 
      M_INFO<<"Roll State 5 is used"; 
      break;
    case STATE6: 
      pimpl->update_values(0.0125,79.228,48.055,5);      //new PID together with PID for height (state 7) and state 6 for pitch
      M_INFO<<"Roll State 6 is used"; 
      break;
    case STATE7: 
      pimpl->update_values(0.0125,3750,0,0);      //new PID together with PID for height (state 7) and state 6 for pitch
      M_INFO<<"Roll State 7 is used"; 
      break;
    case STATE8: 
      pimpl->update_values(0.0125,650,20,1);      //new PID together with PID for height (state 7) and state 6 for pitch
      M_INFO<<"Roll State 8 is used"; 
      break;
  }       
  //pimpl = new PIDImpl(dt,Kp,Kd,Ki);
}
void PID::set_PID_pitch(structures::PIDState state){
 // k->PID_pitch=STATE5;
        
  switch(state){
    case STATE1:
      pimpl->update_values(0.0125,0,0,0);
      M_INFO<<"Pitch State 1 is used";
      break;
    case STATE2:
      pimpl->update_values(0.0125,842.472,236.236,735.936); 
      M_INFO<<"Pitch State 2 is used";
      break;
    case STATE3: 
      pimpl->update_values(0.0125,5240.6,693.258,8623.082); 
      M_INFO<<"Pitch State 3 is used";
      break;
    case STATE4: 
      pimpl->update_values(0.0125,1727.364,1091.903,682.057); 
      M_INFO<<"Pitch State 4 is used";
      break;
    case STATE5: 
      pimpl->update_values(0.0125,0,0,0); 
      M_INFO<<"Pitch State 5 is used";
      break;
    case STATE6: 
      pimpl->update_values(0.0125,1087.349,620.929,0); 
      M_INFO<<"Pitch State 6 is used";
      break;
  }     
    //pimpl = new PIDImpl(dt,Kp,Kd,Ki,Fc);
}
void PID::set_PID_height(structures::PIDState state){
//  k->PID_height=STATE6; //STATE6;    
  switch(state){
      case STATE1: 
        pimpl->update_values(0.0125,0,0,0); 
        M_INFO<<"Height State 1 is used";
        break; 
      case STATE2: 
        pimpl->update_values(0.0125,22.079,2.298,45.602); 
        M_INFO<<"Height State 2 is used";
        break;
      case STATE3: 
        pimpl->update_values(0.0125,402.336,127.467,27.604); 
        M_INFO<<"Height State 3 is used";
        break;
      case STATE4: 
        pimpl->update_values(0.0125,47.138,76.147,1.196); 
        M_INFO<<"Height State 4 is used";
        break;
      case STATE5: 
        pimpl->update_values(0.0125,23.950,69.395,0); 
        M_INFO<<"Height State 5 is used";
        break;
      case STATE6:
        pimpl->update_values(0.0125,0,0,0); 
        M_INFO<<"Height State 6 is used";
        break;
      case STATE7: 
        pimpl->update_values(0.0125,22.526,34.403,0);    //new state, also at 20km/h together with roll state 6 and pitch state 6
        M_INFO<<"Height State 7 is used";
        break;
  }        
    //pimpl = new PIDImpl(dt,Kp,Kd,Ki,Fc);
}
float PID::calculate( float reference, float pv ) {
  if(pimpl!=NULL){  
    return pimpl->calculate(reference,pv);
  } else {
    M_WARN<<"CALCULATING WITHOUT INITIALIZING PIDImpl";
  }
}

PIDValues PID::get_PID_values() {
  return pimpl->get_PID_values();
 }


PID::~PID() {
    delete pimpl;
}



/**
 * Implementation
 */
PIDImpl::PIDImpl( float dt, float Kp, float Kd, float Ki) 
  : dt_(dt), Kp_(Kp), Kd_(Kd), Ki_(Ki), pre_error_(0), integral_(0)
{
}

void PIDImpl::update_values( float dt, float Kp, float Kd, float Ki) {
  dt_ = dt;
  Kp_ = Kp;
  Kd_ = Kd;
  Ki_ = Ki;
}


/*The input for the calculate function will be the reference and the 'previous 
 * value (pv)' that comes in reality from the sensors. This data is stored in 
 * DataStore so we should add these values. This is different for all the PID 
 * controllers   
*/
float PIDImpl::calculate( float reference, float pv )
{
    
    // Calculate error
    float error = reference - pv;

    // Proportional term
    float Pout = Kp_ * error;

    // Integral term
    integral_ += error * dt_;
    float Iout = Ki_ * integral_;

    // Derivative term
    double derivative = (error - pre_error_) / dt_;
    double Dout = Kd_ * derivative;
    //float Dout = Kd_ * (Fc_/(1+Fc_ * integral_));
    
    
    // Calculate total output
    float output = Pout + Iout + Dout;

    // Save error to previous error
    pre_error_ = error;
    
    PID_values_.Dout = Dout;
    PID_values_.Iout = Iout;
    PID_values_.Pout = Pout;
    PID_values_.intergral = integral_;

    return output;
}

PIDValues PIDImpl::get_PID_values() {
  return PID_values_;
}



PIDImpl::~PIDImpl()
{
}