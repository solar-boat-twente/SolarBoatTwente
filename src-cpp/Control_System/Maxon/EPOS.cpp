#include "EPOS.h"
#include "../../../lib-cpp/Canbus/canbus.h"
#include "../../../lib-cpp/ADAM/ADAM.hpp"

#define EPOS_DEBUG

#include <time.h>
#include <iostream>
#include <limits>
#include <cstring>

#include "../../../lib-cpp/Debugging/easy_debugging.hpp"
#include "EPOS.h"
#include "../../../lib-cpp/Canbus/canbus.h"
#include "../../../lib-cpp/ADAM/ADAM.hpp"

using namespace std;
using namespace MIO;
using namespace control;


std::string EPOS::file_name = " ";

/* -----------------------------------------------------------------------------
You are going to open the function create_CAN_message. In this function, a 
message is made that consists of the right NODE_ID, Length and the data. 
This data comes from the data that is written in the .h file. 
MessageOut is a CAN message that is part of the funtion create_CAN_msg and 
will be filled with the right data. 
can.write is a function from mbed and gives a 0 or 1 as output:
    0 when nothing is written on the CAN line
    1 when a message is written on the CAN line
----------------------------------------------------------------------------- */
 
/* -----------------------------------------------------------------------------
With the HOMING function you can home the motor. There are 2 differnt sorts of 
homing: 
    - 0xFD: -3: Current treshold positive speed (first in the direction of 
    the motor and then to the end to go to the defined position).
    - 0xFC: -4: Current treshold negative speed (this is what we are going 
    to use in the real boat: first to the end of then back in the direction 
    to the motor).
For homing different steps have to be executed in the right order. Foo is used 
as a counter to check if all the steps are executed. When can.write is 
succesfull (when there is a message on the CAN line) foo increases with 1. To 
check if all the messages are corrrect, we check it with pc.printf. In tera term
you do not always get a message if homing is succesfull or failed because the 
4th byte is already 0, but the 5th is 81 so you are not in the situations where
the output is succes or failed. 
    
can.read? 
----------------------------------------------------------------------------- */ 
canmsg_t EPOS::create_CAN_msg(int id, int length, const uint8_t data[]){
  canmsg_t output;
  output.id = id;
  output.length = length;
  memcpy(output.data, data, length);
  return output;
  
}
EPOS::EPOS(CANbus * can, UI::ADAM * adam, int node_id, DataStore * const control_data) 
  : canbus_(can), 
    adam_6017(adam), 
    control_data_(control_data) 
{      
  node_id_ = node_id;
  write_id_ = 0x600 + node_id;
  read_id_ = 0x580 + node_id;

  moving_allowed = false;
  
  build_CAN_messages_();
  
  if (file_name == " "){
    std::time_t now = time(nullptr);
    file_name = "/root/logfiles/epos_" + std::to_string(now) + ".txt";
  }
  
  file_.open(file_name, std::ios::app);  
}

EPOS::~EPOS(){
  file_.close();
}

void EPOS::build_CAN_messages_() {
//  shutdown_.id = 0x600 + NODE_ID;
//  shutdown_.length = 8;
//  memcpy(shutdown_.data,kEposShutdownMessage, 8);
//  
//  switch_on_and_enable_.id = 0x600+NODE_ID;
//  switch_on_and_enable_.length = 8;
//  memcpy(switch_on_and_enable_.data,kEposSwitchOnMessage, 8);
//  
//  get_status_word.id = 0x600+NODE_ID;
//  get_status_word.length = 4;
//  memcpy(get_status_word.data,kEposGetStatusMessage, 4);
//  
//  set_homing_mode_.id = 0x600+NODE_ID;
//  set_homing_mode_.length = 8;
//  memcpy(&set_homing_mode_.data[0],(char*)kEposHomingModeMessage, 8);
//  
//  set_homing_method_positive_.id = 0x600+NODE_ID;
//  set_homing_method_positive_.length = 8;
//  memcpy(&set_homing_method_positive_.data[0],(char*)kEposHomingPositiveMessage, 8);
//  
//  set_homing_method_negative_.id = 0x600+NODE_ID;
//  set_homing_method_negative_.length = 8;
//  memcpy(&set_homing_method_negative_.data[0],(char*)kEposHomingNegativeMessage, 8);
//  
//  start_homing_.id = 0x600+NODE_ID;
//  start_homing_.length = 8;
//  memcpy(&start_homing_.data[0],(char*)kEposStartHomingMessage, 8);
//  
//  set_position_mode_.id = 0x600+NODE_ID;
//  set_position_mode_.length = 8;
//  memcpy(&set_position_mode_.data[0],(char*)kEposSetPositionModeMessage, 8);
//  
//  start_absolute_position_.id = 0x600+NODE_ID;
//  start_absolute_position_.length = 8;
//  memcpy(&start_absolute_position_.data[0],(char*)kEposAbsolutePositionMessage, 8);
//  
//  clear_faults_.id = 0x600+NODE_ID;
//  clear_faults_.length = 8;
//  memcpy(&clear_faults_.data[0],(char*)kEposClearFaultMessage, 8);
  
  
  shutdown_ = create_CAN_msg(write_id_, 8, kEposShutdownMessage);
  
  switch_on_and_enable_ = create_CAN_msg(write_id_, 8, kEposSwitchOnMessage);
  get_status_word = create_CAN_msg(write_id_, 4, kEposSwitchOnMessage);
  set_homing_mode_ = create_CAN_msg(write_id_, 8, kEposHomingModeMessage);
  set_homing_method_positive_ = create_CAN_msg(write_id_, 8, kEposHomingPositiveMessage);
  set_homing_method_negative_ = create_CAN_msg(write_id_, 8, kEposHomingNegativeMessage);
  start_homing_ = create_CAN_msg(write_id_, 8, kEposStartHomingMessage);
  clear_faults_ = create_CAN_msg(write_id_, 8, kEposStartHomingMessage);
  
  set_position_mode_ = create_CAN_msg(write_id_, 8, kEposSetPositionModeMessage);
  start_absolute_position_ = create_CAN_msg(write_id_, 8, kEposAbsolutePositionMessage);

 }

void EPOS::start_homing(bool home_positive, short int delay){  
  
  //One shall first clear the faults and wait for this to be done 
  canbus_->write_can(clear_faults_);
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  
  //One shall put the EPOS in homing mode and wait a bit
  canbus_->write_can(set_homing_mode_);        
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  
  //One shall home either negative or positive depending on the input
  if(home_positive) {
    canbus_->write_can(set_homing_method_positive_);
  } else {
    canbus_->write_can(set_homing_method_negative_);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  
  //One shall turn of the EPOS and wait a bit
  canbus_->write_can(shutdown_);      
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  
  // One shall now turn the EPOS back on and enable it.
  canbus_->write_can(switch_on_and_enable_);
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  
  // Finally homing shall start
  canbus_->write_can(start_homing_);
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}

bool EPOS::check_homing(int delay){
    
  for (int i = 0; i<100; i++) {   
    // Setup getting the status word from EPOS and wait 1 second
    canbus_->write_can(get_status_word);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    
    canmsg_t buffer;
    canbus_->read_can(read_id_, &buffer);
    
    if (buffer.data[5] == 0x95){ // case home found
        M_OK<<"Homing NODE: "<<node_id_<<" Successful!!";
        moving_allowed = true;
        file_<<"homing done"<< flush;
        return true;
    } else if ( !(buffer.data[4] == 0x37)){ // case homing failed
        M_WARN<<"NODE: "<<node_id_<<" FAILED!";
        moving_allowed=false;
    }  
  } 
  M_ERR<<"FAILED TO HOME NODE: "<<node_id_;
  return false;
}
    
/* -----------------------------------------------------------------------------
In this boolean funtion, the position is set and the axis is moving to the new 
absolute position with the maximum acceleration and maximum velocity without a 
particular traject. 
With tel we built in a test to see if all the can messages are written on the 
CAN line. 
----------------------------------------------------------------------------- */

void EPOS::start_position_mode(short int delay){
    //int tel=0;
  
  if (moving_allowed==true) {
    canbus_->write_can(clear_faults_);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    
    canbus_->write_can(set_position_mode_);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    
    canbus_->write_can(shutdown_);      
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));

    canbus_->write_can(switch_on_and_enable_);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  } else {
    M_WARN<<"NODE "<<node_id_<<" NOT YET READY WITH HOMING";
  }
  
}  //end of StartPositionMode 

float EPOS::get_angle_from_podmeter() {
  float button_position = (float)adam_6017->read_counter(5)/numeric_limits<uint16_t>::max() - 0.5;
  float button_quartercounts = button_position * kPositionButtonQuartercountMultiplier;
  return button_quartercounts;

}

void EPOS::move(){
    
  float quartercounts;
  
  DataStore::AngleWings alphas = control_data_->GetWingData();

  float user_quatercircles = get_angle_from_podmeter();

    //366889 qc bij 1 rad hoekverdraaiing
  //float quartercircles= (-366889/alphas.Wing_right);

  switch (node_id_) {
    case 1:   //maximale hoeveelheid qc die we positief kunnen maken is 1000000 dit hoort bij 0.5 user_quartercircles //node 1 is rechts, node 2 is links
      quartercounts = angle2quartercounts_(alphas.Wing_right, MIN_ANGLE_RIGHT, MIN_ANGLE_RIGHT) ;
      file_<<"RIGHT: "<<node_id_<<" Alpha: "<<alphas.Wing_right <<" | Position: "<<quartercounts<<flush;
      break;
      
    case 2:
      quartercounts = angle2quartercounts_(alphas.Wing_left, MIN_ANGLE_LEFT, MIN_ANGLE_RIGHT);
      file_<<"\tLEFT: "<<node_id_<<"Alpha: "<<alphas.Wing_left <<" | Position: "<<quartercounts<<"\n"<<flush;
      break;
      
    case 4:
      quartercounts = quartercounts2quartercounts_(user_quatercircles, MIN_ANGLE_BACK, MAX_ANGLE_BACK);//quartercircles + ((alphas.Wing_back*180/3.1415)*125000);   //absolute positie en start direct
      file_<<"\tBACK: "<<node_id_<<"Alpha: "<<alphas.Wing_back <<" | Position: "<<quartercounts<<"\n"<<flush;
      break;
      
    default:
      M_WARN<<"INVALID NODE!";
      break;
  }
    
    
      
    /* -------------------------------------------------------------------------
    Unions allow one portion of memory to be accessed as different data types. 
    Its declaration and use is similar to the one of structures, but its 
    functionality is totally different.
    ------------------------------------------------------------------------- */
    
    /* -------------------------------------------------------------------------
    The data that is sent to determine how many quartercircles have to be turned, 
    consists of 4 bytes -> byte 0 = 0x23.
    The Position Mode Setting Value has object 0x2062-00(byte1=0x62, byte2=0x20,
    byte3=0x00).
    The data is put in byte 4, 5, 6 and 7 and comes from the union Quaters_Union
    ------------------------------------------------------------------------- */
    //cout << "Called MOVE()" << endl;

  four_bytes absolute_position = {(int)quartercounts};
    
  canmsg_t move_message = build_move_message_(absolute_position.byte);
  canbus_->write_can(move_message);


  canbus_->write_can(start_absolute_position_);
}

canmsg_t EPOS::build_move_message_(uint8_t position_in_bytes[]) {

  uint8_t move_data[8];
  move_data[0] = 0x23;
  move_data[1] = 0x7A;  //0x62;
  move_data[2] = 0x60;  //0x20;
  move_data[3] = 0x00;
  move_data[4] = position_in_bytes[0];
  move_data[5] = position_in_bytes[1];
  move_data[6] = position_in_bytes[2];
  move_data[7] = position_in_bytes[3];  
  
  move_message.id = 0x600+node_id_;
  move_message.length = 8;
  memcpy(move_message.data,move_data, 8);
  return move_message;
}
/**
 * Go from angle in radians to angle in degrees
 * @param radians
 * @return float Angle in degrees
 */
float EPOS::rad2deg_(float radians) {
  return radians*180/3.1415;
}

/**
 * Go from angle in radians to amount of quartercounts, takes the max and min into account
 * @param radians The required angle in radians
 * @param min The minimum amount of quartercounts (!!) 
 * @param max The maximum amount of quartercounts (!!)
 * @return float number of quartercounts
 */
float EPOS::angle2quartercounts_(float radians, const int min, const int max) {
  float quartercounts =  rad2deg_(radians)*kQuartercountMultiplier;
  if (quartercounts < min) {
    quartercounts = min;
  } else if (quartercounts > max) {
    quartercounts = max;
  }
  return quartercounts;
}

/**
 * Similar to angle2quartercounts but without the conversion from angle to quartercounts
 * @param quartercounts
 * @param min
 * @param max
 * @return 
 */
float EPOS::quartercounts2quartercounts_(float quartercounts, const int min, const int max) {
  if (quartercounts < min) {
    quartercounts = min;
  } else if (quartercounts > max) {
    quartercounts = max;
  }
  return quartercounts;
}
           
       