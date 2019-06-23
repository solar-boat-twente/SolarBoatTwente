/* 
 * File:   PID_caller.cpp
 * Author: arjan
 * 
 * Created on 11 maart 2019, 12:23
 */
//#include "structures.h"

#include "../../../lib-cpp/Debugging/easy_debugging.hpp"

#include "pid.h"
#include "PID_caller.h"

using namespace MIO;
using namespace structures;
using namespace control;
using namespace std;

MIO::control::PID_caller::PID_caller(DataStore * const control_data) : control_data_(control_data){
  pid_roll = new PID();
  pid_height = new PID();
  pid_pitch = new PID();
  file_.open("/root/logfiles/pd_log_test_06_07.csv", std::ios::app);
}

MIO::control::PID_caller::~PID_caller() {
  delete pid_height;
  delete pid_pitch;
  delete pid_pitch;
  file_.close();
}

void MIO::control::PID_caller::PID_in(structures::FlyMode fly_mode){ 
  DataStore::RealData data_from_complementary_filter = control_data_->GetComplementaryData();
  
  DataStore::PIDDataTotal PIDData;
  
  pid_roll->set_PID_roll(STATE7);
  PIDData.Force_roll = pid_roll->calculate(0, data_from_complementary_filter.Real_roll);

  pid_pitch->set_PID_pitch(STATE1);            //0, val = reference, previous value
  PIDData.Force_pitch = pid_pitch->calculate(0, data_from_complementary_filter.Real_pitch);
  
  switch(fly_mode) {
    case NO_FLY: 
      PIDData.Force_height = pid_height->calculate(0, data_from_complementary_filter.Real_height); 
      M_INFO<<"USING STATE NO_FLY";
      break;
    case FLY: 
      PIDData.Force_height = pid_height->calculate(0.3, data_from_complementary_filter.Real_height);  
      M_INFO<<"USING STATE FLY";
      break;
    case BRIDGE: 
      PIDData.Force_height = pid_height->calculate(0.1, data_from_complementary_filter.Real_height); 
      M_INFO<<"USING STATE BRIDGE";
      break;
    case SLALOM: 
      PIDData.Force_height = pid_height->calculate(-0.05, data_from_complementary_filter.Real_height); 
      M_INFO<<"USING STATE SLALOM";
      break;
  }
  M_INFO<<"PID INPUT VALUES:\n"<<"Roll: "<<data_from_complementary_filter.Real_roll<<" | Pitch: "
      << data_from_complementary_filter.Real_pitch<<" | Height: "<<data_from_complementary_filter.Real_height;
  M_INFO<<"\nPID FORCES: \n"<<"Roll: "<<PIDData.Force_roll<<" | Pitch: "
      << PIDData.Force_pitch<<" | Height: "<<PIDData.Force_height;
  
  M_INFO<<"SPLIT VALUES HEIGHT: \n"
      <<"Pout: "<<pid_height->get_PID_values().Pout<<" | Iout: "<<pid_height->get_PID_values().Iout
      <<" | Dout: "<< pid_height->get_PID_values().Dout;
  
  file_<<"SPLIT VALUES ROLL: "
    <<"Pout: "<<pid_roll->get_PID_values().Pout<<"\tIout: "<<pid_roll->get_PID_values().Iout<<"\tIntegral: "<<pid_roll->get_PID_values().intergral
    <<"\tDout: "<< pid_roll->get_PID_values().Dout<< "\tRoll: "<<data_from_complementary_filter.Real_roll<<"\n"<< flush  ;
  file_<<"SPLIT VALUES HEIGHT: "
    <<"Pout: "<<pid_height->get_PID_values().Pout<<"\tIout: "<<pid_height->get_PID_values().Iout<<"\tIntegral: "<<pid_height->get_PID_values().intergral
    <<"\tDout: "<< pid_height->get_PID_values().Dout<< "\tHeight: " <<data_from_complementary_filter.Real_height <<"\n"<< flush  ;

  //Finally copy the PIDData into the DataStore object
  control_data_ -> PutPIDData(&PIDData);
}
    

