/* 
 * File:   PID_caller.cpp
 * Author: arjan
 * 
 * Created on 11 maart 2019, 12:23
 */
#include "structures.h"
#include "pid.h"
#include "PID_caller.h"
#include <stdio.h>  
using namespace MIO;
using namespace structures;
using namespace control;
using namespace std;
void PID_caller::Setting_PID(){
//    pid_roll.Kp=0.42110;
//    pid_roll.Kd=9.72072;
//    pid_roll.Ki=0.003107;
//    pid_roll.Fc=0.960869;
//    UserInput::ControlUserInput States;
//    UserInput::ControlUserInput * control =&States;
    
//    switch (control->PID_roll){
//       case STATE1: pid_roll.Kp=0.42110,pid_roll.Kd=9.72072,pid_roll.Ki=0.003107,pid_roll.Fc=0.960869;break;
//       //case STATE2: pid_roll.Kp=0.3,pid_roll.Kd=10,pid_roll.Ki=0.004,pid_roll.Fc=1 ;break;
//    }
    
}
PID_caller::PID_caller(){
}

PID_caller::~PID_caller() {
}
void PID_caller::PID_in(){ 
    DataStore::RealData inputdata = m_complementary_data-> GetComplementaryData();
    DataStore::PIDDataTotal PIDData;
    PID * pid = new PID;
    pid->PIDRoll();
    PIDData.Force_roll = pid->calculate(0, inputdata.Real_roll);
                    
    //control::PID pitch = control::PID(0.05, 2603.070, 1742.409, 822.026, 429.744);   
    pid->PIDPitch();            //0, val = reference, previous value
    PIDData.Force_pitch = pid->calculate(0, inputdata.Real_pitch);
        
    
    //control::PID height = control::PID(0.05, 145.200, 5.761, 86.764, 2.327);
    //PIDData.Force_height = calculate(0, inputdata.Real_height);
                //0, val = reference, previous value
                //PIDData.Force_height = height.calculate(0.3, inputdata.Real_height);
    pid->PIDHeight();
    p->fly_mode = NO_FLY;
        switch(p->fly_mode)
    {
            case NO_FLY : PIDData.Force_height = pid->calculate(0, inputdata.Real_height); 
            printf("state NO_FLY \r\n");break;
            case FLY    : PIDData.Force_height = pid->calculate(0.3, inputdata.Real_height);  
            printf("state FLY \r\n"); break;
            case BRIDGE : PIDData.Force_height = pid->calculate(0.1, inputdata.Real_height); 
            printf("state BRIDGE \r\n"); break;
            case SLALOM : PIDData.Force_height = pid->calculate(-0.05, inputdata.Real_height); 
            printf("state SLALOM \r\n");break;
    }
     printf ("PID in roll %f \r\n", inputdata.Real_roll) ;
     printf ("PID in pitch %f \r\n", inputdata.Real_pitch) ;
     printf ("PID in hoogte %f \r\n", inputdata.Real_height) ;
     printf ("PID force roll %f \r\n", PIDData.Force_roll) ;
     printf ("PID force pitch %f \r\n", PIDData.Force_pitch) ;
     printf ("PID force hoogte %f \r\n", PIDData.Force_height) ;
    m_PID_data-> PutPIDData(&PIDData);
}
    

