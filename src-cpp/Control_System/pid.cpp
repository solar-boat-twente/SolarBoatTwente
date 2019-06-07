/* 
 * File:   pid.cpp
 * Author: User
 * 
 * Created on 27 februari 2019, 10:28
 */
#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;
using namespace MIO;
using namespace structures;
using namespace control;
//float dt, float Kp, float Kd, float Ki, float Fc
void control::PID::PIDRoll(){
        k->PID_roll=STATE8;//STATE3;
        
    switch(k->PID_roll){
        case STATE1 : pimpl = new PIDImpl(0.0125,237.597,27.078,-132.816,1.424); //laf
        printf("roll state 1 word gebruikt \r\n"); break;
        case STATE2 : pimpl = new PIDImpl(0.0125,21.218,1.289,48.819,0.273);  //heel laf
        printf("roll state 2 word gebruikt\r\n"); break;
        case STATE3 : pimpl = new PIDImpl(0.0125,414.957,-326.505,80.103,0.907);  // zeer aggressief
        printf("roll state 3  word gebruikt\r\n"); break;
        case STATE4 : pimpl = new PIDImpl(0.01251,269.078,-169.860,48.055,1.584); 
        printf("roll state 4  word gebruikt\r\n"); break;
        case STATE5 : pimpl = new PIDImpl(0.0125,79.228,5,0,100); 
        printf("roll state 5  word gebruikt\r\n"); break;
        case STATE6 : pimpl = new PIDImpl(0.0125,79.228,48.055,5,100);      //new PID together with PID for height (state 7) and state 6 for pitch
        printf("roll state 5  word gebruikt\r\n"); break;
        case STATE7 : pimpl = new PIDImpl(0.0125,0,0,0,0);      //new PID together with PID for height (state 7) and state 6 for pitch
        printf("roll state 5  word gebruikt\r\n"); break;
        case STATE8 : pimpl = new PIDImpl(0.0125,650,20,0,1);      //new PID together with PID for height (state 7) and state 6 for pitch
        printf("roll state 5  word gebruikt\r\n"); break;
    }       
    //pimpl = new PIDImpl(dt,Kp,Kd,Ki,Fc);
}
void control::PID::PIDPitch(){
        k->PID_pitch=STATE5;
        
    switch(k->PID_pitch){
        case STATE1 : pimpl = new PIDImpl(0.0125,10000,100,16000,10);
        printf("pitch state 1 word gebruikt \r\n");break;
        case STATE2 : pimpl = new PIDImpl(0.0125,842.472,236.236,735.936,307.568); 
        printf("pitch state 2 word gebruikt \r\n");break;
        case STATE3 : pimpl = new PIDImpl(0.0125,5240.6,693.258,8623.082,26.629); 
        printf("pitch state 3 word gebruikt\r\n"); break;
        case STATE4 : pimpl = new PIDImpl(0.0125,1727.364,1091.903,682.057,490.536); 
        printf("pitch state 4 word gebruikt \r\n"); break;
        case STATE5 : pimpl = new PIDImpl(0.0125,0,0,0,0); 
        printf("pitch state 5 word gebruikt \r\n"); break;
        case STATE6 : pimpl = new PIDImpl(0.0125,1087.349,620.929,0,295.061); 
        printf("pitch state 6 word gebruikt \r\n"); break;
        
    }     
    //pimpl = new PIDImpl(dt,Kp,Kd,Ki,Fc);
}
void control::PID::PIDHeight(){
        k->PID_height=STATE6; //STATE6;
        
    switch(k->PID_height){
        case STATE1 : pimpl = new PIDImpl(0.0125,145.200,5.761,86.764,2.327); 
        printf("height state 1 word gebruikt \r\n");break; 
        case STATE2 : pimpl = new PIDImpl(0.0125,22.079,2.298,45.602,1.786); 
        printf("height state 2 word gebruikt \r\n");break;
        case STATE3 : pimpl = new PIDImpl(0.0125,402.336,127.467,27.604,3.994); 
        printf("height state 3 word gebruikt\r\n"); break;
        case STATE4 : pimpl = new PIDImpl(0.0125,47.138,76.147,1.196,0.146); 
        printf("height state 4 word gebruikt \r\n"); break;
        case STATE5 : pimpl = new PIDImpl(0.0125,23.950,69.395,0,0.090); 
        printf("height state 5 word gebruikt \r\n"); break;
        case STATE6 : pimpl = new PIDImpl(0.0125,0,0,0,0); 
        printf("height state 5 word gebruikt \r\n"); break;
        case STATE7 : pimpl = new PIDImpl(0.0125,22.526,34.403,0,0.070);    //new state, also at 20km/h together with roll state 6 and pitch state 6
        printf("height state 5 word gebruikt \r\n"); break;
    }        
    //pimpl = new PIDImpl(dt,Kp,Kd,Ki,Fc);
}
float control::PID::calculate( float reference, float pv )
{
    return pimpl->calculate(reference,pv);
}
control::PID::~PID() 
{
    delete pimpl;
}

/**
 * Implementation
 */
control::PIDImpl::PIDImpl( float dt, float Kp, float Kd, float Ki, float Fc ) :
    _dt(dt),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _Fc(Fc),
    _pre_error(0),
    _integral(0)
{
}
/*The input for the calculate function will be the reference and the 'previous 
 * value (pv)' that comes in reality from the sensors. This data is stored in 
 * DataStore so we should add these values. This is different for all the PID 
 * controllers   
*/
float control::PIDImpl::calculate( float reference, float pv )
{
    
    // Calculate error
    float error = reference - pv;

    // Proportional term
    float Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    float Iout = _Ki * _integral;

    // Derivative term
    //double derivative = (error - _pre_error) / _dt;
    //double Dout = _Kd * derivative;
    float Dout = _Kd * (_Fc/(1+_Fc* _integral));
    // Calculate total output
    float output = Pout + Iout + Dout;

    // Save error to previous error
    _pre_error = error;

    return output;
}

control::PIDImpl::~PIDImpl()
{
}
