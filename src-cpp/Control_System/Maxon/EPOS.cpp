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
canmsg_t EPOS::create_CAN_msg(int id, int length, uint8_t data[]){
  canmsg_t output;
  output.id = id;
  output.length = length;
  memcpy(&output.data, data, length);
  return output;
  
}

void EPOS::build_CAN_messages_() {
  
  shutdown_ = create_CAN_msg(write_id_, 8, EPOS_SHUTDOWN);
  switch_on_and_enable_ = create_CAN_msg(write_id_, 8, EPOS_SWITCH_ON);
  get_status_word = create_CAN_msg(write_id_, 4, EPOS_GET_STATUS);
  set_homing_mode_ = create_CAN_msg(write_id_, 8, EPOS_HOMING_MODE);
  set_homing_method_positive_ = create_CAN_msg(write_id_, 8, EPOS_SET_HOMING_POSITIVE);
  set_homing_method_negative_ = create_CAN_msg(write_id_, 8, EPOS_SET_HOMING_NEGATIVE);
  start_homing_ = create_CAN_msg(write_id_, 8, EPOS_START_HOMING);
  clear_faults_ = create_CAN_msg(write_id_, 8, EPOS_CLEAR_FAULT);
  set_position_mode_ = create_CAN_msg(write_id_, 8, EPOS_SET_POSITION_MODE);
  start_absolute_position_ = create_CAN_msg(write_id_, 8, EPOS_ABSOLUTE_POSITION);

  
 }



void EPOS::start_homing(bool home_positive){  //foo is een teller, als can.write lukt komt er een 1 uit en als het faalt een 0. Uiteindelijk moet foo in dit geval dus 5 zijn.  
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

  switch (NODE_ID) {
    case 1:
      quartercircles=quartercircles + ((alphas.Wing_left*180/3.1415)*10000)+user_quatercircles;
      if (quartercircles < MIN_ANGLE_LEFT) {
        quartercircles = MIN_ANGLE_LEFT;
      } else if (quartercircles > MAX_ANGLE_LEFT) {
        quartercircles = MAX_ANGLE_LEFT;
      }
      M_DEBUG<<"NODE: "<<NODE_ID<<"\nAlpha Left: "<<alphas.Wing_left <<" | POSITION LEFT: "<<quartercircles;
      break;
      
    case 2:
      quartercircles=quartercircles + ((alphas.Wing_right*180/3.1415)*10000);
      if (quartercircles < MIN_ANGLE_RIGHT) {
        quartercircles = MIN_ANGLE_RIGHT;
      } else if (quartercircles > MAX_ANGLE_RIGHT) {
        quartercircles = MAX_ANGLE_RIGHT;
      }
      M_DEBUG<<"NODE: "<<NODE_ID<<"\nAlpha Right: "<<alphas.Wing_right <<" | Position Right: "<<quartercircles;
      break;
      
    case 4:
      quartercircles=quartercircles + ((alphas.Wing_back*180/3.1415)*125000);   //absolute positie en start direct
      if (quartercircles < MIN_ANGLE_BACK) {
        quartercircles = MIN_ANGLE_BACK;
      } else if (quartercircles > MAX_ANGLE_BACK) {
        quartercircles = MAX_ANGLE_BACK;
      }
      M_DEBUG<<"NODE: "<<NODE_ID<<"\nAlpha Back: "<<alphas.Wing_back <<" | Position Back: "<<quartercircles;
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
    
  canmsg_t move_message = build_move_message_(absolute_position.byte);

  canbus->write_can(&move_message);

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
  
  return create_CAN_msg(write_id_, 8, move_data);
 }

        
              
       