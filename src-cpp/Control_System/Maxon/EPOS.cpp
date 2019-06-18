#include "EPOS.h"
#include "../../../lib-cpp/Canbus/canbus.h"
#include "../../../lib-cpp/ADAM/ADAM.hpp"
#define EPOS_DEBUG

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <thread>
#include <chrono>
#include <iostream>

#include "../../../lib-cpp/Debugging/easy_debugging.hpp"
#include "EPOS.h"
#include "../../../lib-cpp/Canbus/canbus.h"
#include "../../../lib-cpp/ADAM/ADAM.hpp"

using namespace std;
using namespace MIO;
using namespace Control;
namespace MIO{
namespace Control{

// TODO: make this const

uint8_t EPOS_SHUTDOWN[8] = {0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};        //voorkomen dat je niet nog in een andere mode zit
    
uint8_t EPOS_SWITCH_ON[8] = {0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00}; //enable de motorcontroller (groene ledje constant aan)

uint8_t EPOS_HOMING_MODE[8] = {0x2f, 0x60, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};     //aan de 0x6060-00 geef je aan dat je nu gaat homen wat de waarde 06 heeft in de data bytes
   
uint8_t EPOS_SET_HOMING_POSITIVE[8] = {0x2F,0x98,0x60,0x00,0xFD,0x00,0x00,0x00};  //bij de index 0x6098-00 geef je aan welke homing method je gaat gebruiken: 0xFD = -3 = current treshold positive speed

uint8_t EPOS_SET_HOMING_NEGATIVE[8] = {0x2F,0x98,0x60,0x00,0xFC,0x00,0x00,0x00};  //bij de index 0x6098-00 geef je aan welke homing method je gaat gebruiken: 0xFC = -4 = current treshold negative speed
    
uint8_t EPOS_START_HOMING[8] = {0x2B,0x40,0x60,0x00,0x1F,0x00,0x00,0x00}; //om te beginnen met homen geef je het controlword de waarde 0x001F om het homen te starten 

uint8_t EPOS_GET_STATUS[4] = {0x40,0x41,0x60,0x00};  //0x40, want ccs is 2: je upload iets van de controller om te kijken of het gelukt is. 

uint8_t EPOS_CLEAR_FAULT[8] = {0x2B,0x40,0x60,0x00,0x80,0x00,0x00,0x00}; //Clear fault: 0x0080 versturen naar 0x6040-00    

uint8_t MOVE_MESSAGE[8] = {0x23, 0x7a, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
/* -----------------------------------------------------------------------------
Additional EPOS commands that you need for going to a position, you can find 
those in chapter 8.7 from the application node. 
----------------------------------------------------------------------------- */
uint8_t EPOS_SET_POSITION_MODE[8] = {0x2F,0x60,0x60,0x00,0x01,0x00,0x00,0x00};

uint8_t EPOS_ABSOLUTE_POSITION[8] = {0x2B,0x40,0x60,0x00,0x3F,0x00,0x00,0x00};
}
}
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
canmsg_t EPOS::create_CAN_msg(int id, int length, char * data){
  canmsg_t output;
  output.id = id;
  output.length = length;
  memcpy(output.data, data, length);
  return output;
  
}


// TODO: replace memcpy(&foo.x[0], ..) --> memcpy(foo.x, ...)
void EPOS::build_CAN_messages_() {
  shutdown_.id = 0x600 + NODE_ID;
  shutdown_.length = 8;
  memcpy(&shutdown_.data[0],(char*)EPOS_SHUTDOWN, 8);
  
  switch_on_and_enable_.id = 0x600+NODE_ID;
  switch_on_and_enable_.length = 8;
  memcpy(&switch_on_and_enable_.data[0],(char*)EPOS_SWITCH_ON, 8);
  
  get_status_word.id = 0x600+NODE_ID;
  get_status_word.length = 4;
  memcpy(&get_status_word.data[0],(char*)EPOS_GET_STATUS, 4);
  
  set_homing_mode_.id = 0x600+NODE_ID;
  set_homing_mode_.length = 8;
  memcpy(&set_homing_mode_.data[0],(char*)EPOS_HOMING_MODE, 8);
  
  set_homing_method_positive_.id = 0x600+NODE_ID;
  set_homing_method_positive_.length = 8;
  memcpy(&set_homing_method_positive_.data[0],(char*)EPOS_SET_HOMING_POSITIVE, 8);
  
  set_homing_method_negative_.id = 0x600+NODE_ID;
  set_homing_method_negative_.length = 8;
  memcpy(&set_homing_method_negative_.data[0],(char*)EPOS_SET_HOMING_NEGATIVE, 8);
  
  start_homing_.id = 0x600+NODE_ID;
  start_homing_.length = 8;
  memcpy(&start_homing_.data[0],(char*)EPOS_START_HOMING, 8);
  
  set_position_mode_.id = 0x600+NODE_ID;
  set_position_mode_.length = 8;
  memcpy(&set_position_mode_.data[0],(char*)EPOS_SET_POSITION_MODE, 8);
  
  start_absolute_position_.id = 0x600+NODE_ID;
  start_absolute_position_.length = 8;
  memcpy(&start_absolute_position_.data[0],(char*)EPOS_ABSOLUTE_POSITION, 8);
  
  clear_faults_.id = 0x600+NODE_ID;
  clear_faults_.length = 8;
  memcpy(&clear_faults_.data[0],(char*)EPOS_CLEAR_FAULT, 8);
  
//  
//  shutdown_ = create_CAN_msg(write_id_, 8, (char*)EPOS_SHUTDOWN);
//  
//  switch_on_and_enable_ = create_CAN_msg(write_id_, 8, (char*)EPOS_SWITCH_ON);
//  get_status_word = create_CAN_msg(write_id_, 4, (char*)EPOS_GET_STATUS);
//  set_homing_mode_ = create_CAN_msg(write_id_, 8, (char*)Homing_Mode_Data);
//  set_homing_method_positive_ = create_CAN_msg(write_id_, 8, (char*)Homing_Method_Data_Positive);
//  set_homing_method_negative_ = create_CAN_msg(write_id_, 8, (char*)EPOS_SET_HOMING_NEGATIVE);
//  start_homing_ = create_CAN_msg(write_id_, 8, (char*)EPOS_START_HOMING);
//  //clear_faults_ = create_CAN_msg(write_id_, 8, (char*)Clear_Fault_Data);
//  
//  set_position_mode_ = create_CAN_msg(write_id_, 8, (char*)EPOS_SET_POSITION_MODE);
//  start_absolute_position_ = create_CAN_msg(write_id_, 8, (char*)EPOS_ABSOLUTE_POSITION);

  
 }

void EPOS::Homing(){ 
    canmsg_t m_Clear_Fault;
    m_Clear_Fault.id = 0x600+NODE_ID;
    m_Clear_Fault.length = 8;
    memcpy(&m_Clear_Fault.data[0],(char*)Clear_Fault_Data, 8);
    canbus->write_can(&m_Clear_Fault);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
   
    canmsg_t m_Homing_Mode; 
    m_Homing_Mode.id = 0x600+NODE_ID;
    m_Homing_Mode.length = 8;
    memcpy(&m_Homing_Mode.data[0],(char*)Homing_Mode_Data, 8);
    canbus->write_can(&m_Homing_Mode);        
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
   
    canmsg_t m_Homing_Mode_Positive; 
    m_Homing_Mode_Positive.id = 0x600+NODE_ID;
    m_Homing_Mode_Positive.length = 8;
    memcpy(&m_Homing_Mode_Positive.data[0],(char*)Homing_Method_Data_Positive, 8);    
    canbus->write_can(&m_Homing_Mode_Positive);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
   
    canmsg_t m_Shutdown; 
    m_Shutdown.id = 0x600+NODE_ID;
    m_Shutdown.length = 8;
    memcpy(&m_Shutdown.data[0],(char*)Shutdown_Data, 8);    
    canbus->write_can(&m_Shutdown);      
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
   
    canmsg_t m_SwitchOnAndEnable; 
    m_SwitchOnAndEnable.id = 0x600+NODE_ID;
    m_SwitchOnAndEnable.length = 8;
    memcpy(&m_SwitchOnAndEnable.data[0],(char*)Switch_On_And_Enable_Data, 8);  
    canbus->write_can(&m_SwitchOnAndEnable);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
   
    canmsg_t m_StartHoming; 
    m_StartHoming.id = 0x600+NODE_ID;
    m_StartHoming.length = 8;
    memcpy(&m_StartHoming.data[0],(char*)Start_Homing_Data, 8);
    canbus->write_can(&m_StartHoming);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
}

void EPOS::start_homing(bool home_positive){  
  //One shall first clear the faults and wait for this to be done 
  
  canbus->write_can(&clear_faults_);
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  
  //One shall put the EPOS in homing mode and wait a bit
  canbus->write_can(&set_homing_mode_);        
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  
  //One shall home either negative or positive depending on the input
  if(home_positive) {
    canbus->write_can(&set_homing_method_positive_);
  } else {
    canbus->write_can(&set_homing_method_negative_);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  
  //One shall turn of the EPOS and wait a bit
  canbus->write_can(&shutdown_);      
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  
  // One shall now turn the EPOS back on and enable it.
  canbus->write_can(&switch_on_and_enable_);
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  
  // Finally homing shall start
  canbus->write_can(&start_homing_);
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
}

bool EPOS::check_homing(){
    
  for (int i = 0; i<100; i++) {   
    // Setup getting the status word from EPOS and wait 1 second
    canbus->write_can(&get_status_word);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    canmsg_t buffer;
    canbus->read_can(read_id_, &buffer);
    
    if (buffer.data[5] == 0x95){ // case home found
        M_OK<<"Homing NODE: "<<NODE_ID<<" Successful!!";
        moving_allowed = 1;
        file_<<"homing done"<< flush;
        return true;
    } else if ( !(buffer.data[4] == 0x37)){ // case homing failed
        M_WARN<<"NODE: "<<NODE_ID<<" FAILED!";
        moving_allowed=0;
    }  
  } 
  M_ERR<<"FAILED TO HOME NODE: "<<NODE_ID;
  return false;
}
    
/* -----------------------------------------------------------------------------
In this boolean funtion, the position is set and the axis is moving to the new 
absolute position with the maximum acceleration and maximum velocity without a 
particular traject. 
With tel we built in a test to see if all the can messages are written on the 
CAN line. 
----------------------------------------------------------------------------- */

// TODO: make wait times configurable -- constexpr size_t kHomingWaitTime = ...;, or as a parameter
void EPOS::start_position_mode(){
    //int tel=0;
  
  if (moving_allowed=1){
        
    canbus->write_can(&clear_faults_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    canbus->write_can(&set_position_mode_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    canbus->write_can(&shutdown_);      
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    canbus->write_can(&switch_on_and_enable_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  else{
    M_WARN<<"NODE "<<NODE_ID<<" NOT YET READY WITH HOMING";
  }
  
}  //end of StartPositionMode 

// TODO: replace magic numbers --> std::numeric_limits for 65536
float EPOS::get_angle_from_podmeter() {
  float button_position = (float)adam_6017->read_counter(5)/65536 - 0.5;
  
  return button_position * POSITION_BUTTON_QUARTERCIRCLE_MULTIPLIER;

}

void EPOS::move(){
    
  float quartercircles;
  
  DataStore::AngleWings alphas = m_FtoW_data->GetWingData();

  float user_quatercircles = get_angle_from_podmeter();

    //366889 qc bij 1 rad hoekverdraaiing
  //float quartercircles= (-366889/alphas.Wing_right);

  // TODO: refactor into something better --> derive from EPOS, or use parameters/members
  // Use enum class to keep track of which wing this EPOS manages,
  // and set quartercircles appropriately
  switch (NODE_ID) {
    case 1:   //maximale hoeveelheid qc die we positief kunnen maken is 1000000 dit hoort bij 0.5 user_quartercircles //node 1 is rechts, node 2 is links
      quartercircles = ((alphas.Wing_left*180/3.1415)*10000) ;
      if (quartercircles < MIN_ANGLE_LEFT) {
        quartercircles = MIN_ANGLE_LEFT;
      } else if (quartercircles > MAX_ANGLE_LEFT) {
        quartercircles = MAX_ANGLE_LEFT;
      }
      file_<<"NODE: "<<NODE_ID<<" Alpha: "<<alphas.Wing_left <<" | Position: "<<quartercircles<<flush;
      break;
      
    case 2:
      quartercircles = ((alphas.Wing_right*180/3.1415)*10000);
      if (quartercircles < MIN_ANGLE_RIGHT) {
        quartercircles = MIN_ANGLE_RIGHT;
      } else if (quartercircles > MAX_ANGLE_RIGHT) {
        quartercircles = MAX_ANGLE_RIGHT;
      }
      file_<<"\tNODE: "<<NODE_ID<<"Alpha: "<<alphas.Wing_right <<" | Position: "<<quartercircles<<"\n"<<flush;
      break;
      
    case 4:
      quartercircles = user_quatercircles;//quartercircles + ((alphas.Wing_back*180/3.1415)*125000);   //absolute positie en start direct
      cout << "qc achter daadwerkelijk: " << quartercircles << endl;
      if (quartercircles < MIN_ANGLE_BACK) {
        quartercircles = MIN_ANGLE_BACK;
      } else if (quartercircles > MAX_ANGLE_BACK) {
        quartercircles = MAX_ANGLE_BACK;
      }
      file_<<"\tNODE: "<<NODE_ID<<"Alpha: "<<alphas.Wing_back <<" | Position: "<<quartercircles<<"\n"<<flush;
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

  four_bytes absolute_position = {(int)quartercircles};
    
  build_move_message_(absolute_position.byte);

  

  canbus->write_can(&start_absolute_position_);
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
  
  move_message.id = 0x600+NODE_ID;
  move_message.length = 8;
  memcpy(&move_message.data[0],(char*)move_data, 8);
  canbus->write_can(&move_message);
 }

        
              
       