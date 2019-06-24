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
using namespace control;
using namespace structures;
//float dt, float Kp, float Kd, float Ki, float Fc
void PID::set_PID_roll(structures::PIDState state){
  
//  k->PID_roll=STATE8;//STATE3;
  switch(state){
    case STATE1: 
      pimpl->update_values(0.0125,0,0,0); 
      M_INFO<<"Roll State 1 is used"; 
      break;
    case STATE2: 
      pimpl->update_values(0.0125,1500,200,0);      
      M_INFO<<"Roll State 2 is used"; 
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
      pimpl->update_values(0.0125,150,35,0); 
      M_INFO<<"Pitch State 2 is used";
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
      pimpl->update_values(0.0125,1600,100,0); //27000demping toevoegen zodat er tijd is om de achtervleugel bij te stellen
      M_INFO<<"Height State 2 is used";
      break;
    case STATE3: 
      pimpl->update_values(0.0125,900,300,0); 
      M_INFO<<"Height State 3 is used";
      break;
    case STATE4: 
      pimpl->update_values(0.0125,1200,300,0); 
      M_INFO<<"Height State 4 is used";
      break;
    case STATE5: 
      pimpl->update_values(0.0125,1200,200,0); 
      M_INFO<<"Height State 5 is used";
      break;
    case STATE6: 
      pimpl->update_values(0.0125,700,100,0); 
      M_INFO<<"Height State 6 is used";
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