#ifndef DAAN_TEST1_MAXON_H
#define DAAN_TEST1_MAXON_H

#include "../DataStore.h"

#include "../../../lib-cpp/Canbus/canbus.h"
#include "../../../lib-cpp/ADAM/ADAM.hpp"

#include <fstream>

namespace MIO{
namespace Control{

const int POSITION_BUTTON_QUARTERCIRCLE_MULTIPLIER = 6000000;//2000000;//2200000;

const int MIN_ANGLE_LEFT = -100000;
const int MAX_ANGLE_LEFT = 110000;

const int MIN_ANGLE_RIGHT = -100000;
const int MAX_ANGLE_RIGHT = 110000;

const int MIN_ANGLE_BACK = 0;//-2000000;
const int MAX_ANGLE_BACK = 2950000;//1100000;

/* -----------------------------------------------------------------------------
EPOS commands that you need to home, you can find those in chapter 8.3 from the 
application node. 
----------------------------------------------------------------------------- */
extern uint8_t EPOS_SHUTDOWN[8];        //voorkomen dat je niet nog in een andere mode zit
    
extern uint8_t EPOS_SWITCH_ON[8]; //enable de motorcontroller (groene ledje constant aan)

extern uint8_t EPOS_HOMING_MODE[8];     //aan de 0x6060-00 geef je aan dat je nu gaat homen wat de waarde 06 heeft in de data bytes
   
extern uint8_t EPOS_SET_HOMING_POSITIVE[8];  //bij de index 0x6098-00 geef je aan welke homing method je gaat gebruiken: 0xFD = -3 = current treshold positive speed

extern uint8_t EPOS_SET_HOMING_NEGATIVE[8];  //bij de index 0x6098-00 geef je aan welke homing method je gaat gebruiken: 0xFC = -4 = current treshold negative speed
    
extern uint8_t EPOS_START_HOMING[8]; //om te beginnen met homen geef je het controlword de waarde 0x001F om het homen te starten 

extern uint8_t EPOS_GET_STATUS[4];  //0x40, want ccs is 2: je upload iets van de controller om te kijken of het gelukt is. 

extern uint8_t EPOS_CLEAR_FAULT[8]; //Clear fault: 0x0080 versturen naar 0x6040-00    

extern uint8_t MOVE_MESSAGE[8];
/* -----------------------------------------------------------------------------
Additional EPOS commands that you need for going to a position, you can find 
those in chapter 8.7 from the application node. 
----------------------------------------------------------------------------- */
extern uint8_t EPOS_SET_POSITION_MODE[8];

extern uint8_t EPOS_ABSOLUTE_POSITION[8];


/* -----------------------------------------------------------------------------
EPOS commands that you need to home, you can find those in chapter 8.3 from the 
application node. 
----------------------------------------------------------------------------- */
const  unsigned char Shutdown_Data[8] = {0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00};        //voorkomen dat je niet nog in een andere mode zit
    
const unsigned char Switch_On_And_Enable_Data[8] = {0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00}; //enable de motorcontroller (groene ledje constant aan)

const unsigned char Homing_Mode_Data[8] = {0x2F,0x60,0x60,0x00,0x06,0x00,0x00,0x00};     //aan de 0x6060-00 geef je aan dat je nu gaat homen wat de waarde 06 heeft in de data bytes
    
const unsigned char Homing_Method_Data_Positive[8] = {0x2F,0x98,0x60,0x00,0xFD,0x00,0x00,0x00};  //bij de index 0x6098-00 geef je aan welke homing method je gaat gebruiken: 0xFD = -3 = current treshold positive speed

const unsigned char Homing_Method_Data_Negative[8] = {0x2F,0x98,0x60,0x00,0xFC,0x00,0x00,0x00};  //bij de index 0x6098-00 geef je aan welke homing method je gaat gebruiken: 0xFC = -4 = current treshold negative speed
    
const unsigned char Start_Homing_Data[8] = {0x2B,0x40,0x60,0x00,0x1F,0x00,0x00,0x00}; //om te beginnen met homen geef je het controlword de waarde 0x001F om het homen te starten 

const unsigned char Status_Word_Data[4] = {0x40,0x41,0x60,0x00};  //0x40, want ccs is 2: je upload iets van de controller om te kijken of het gelukt is. 

const unsigned char Clear_Fault_Data[8] = {0x2B,0x40,0x60,0x00,0x80,0x00,0x00,0x00}; //Clear fault: 0x0080 versturen naar 0x6040-00    

/* -----------------------------------------------------------------------------
Additional EPOS commands that you need for going to a position, you can find 
those in chapter 8.7 from the application node. 
----------------------------------------------------------------------------- */
const unsigned char Position_Mode_Data[8] = {0x2F,0x60,0x60,0x00,0xFF,0x00,0x00,0x00};

const unsigned char Start_Absolute_Pos[8] = {0x2F,0x40,0x60,0x00,0x3F,0x00,0x00,0x00};

/* -----------------------------------------------------------------------------
Sends a CAN Message to the controller
Returns 1 for succes. May be slow. a faster send message is build into Goto_Pos
@param id The id to set the data to
@param length length of the data to be send
@param data data of the can message
----------------------------------------------------------------------------- */
    

class EPOS{ 
      
  union four_bytes{
      int integer;
      uint8_t byte[4];
  };
  
 public:
  EPOS(CANbus * can, UI::ADAM * adam, int node_id, DataStore * FtoW_data) 
      : canbus(can), adam_6017(adam), m_FtoW_data(FtoW_data) {
        
    NODE_ID = node_id;
    write_id_ = 0x600 + node_id;
    read_id_ = 0x580 + node_id;
    
    moving_allowed = false;
    build_CAN_messages_();
    
    file_.open("/root/logfiles/epos_log_test_06_07.csv",std::ios::app);
      
      
  }
   
      void Homing();
  /** Homes the EPOS with Current mode */
  void start_homing(bool home_positive = true);

  bool check_homing();
  /** With this function the axis is set in the position mode */
  void start_position_mode();

  /** With this function the axis is going to move */
  void move();
    
 private:
  
  canmsg_t create_CAN_msg(int id, int length, char * data);

  
  void build_CAN_messages_();
  
  float get_angle_from_podmeter();
  
  canmsg_t build_move_message_(uint8_t position_in_bytes[]);
  
    /** puts the EPOS in shutdown state */
  canmsg_t shutdown_;

  /** puts the EPOS in Switchon state */
  canmsg_t switch_on_and_enable_;

  /** Requests a statusword from the EPOS */
  canmsg_t get_status_word;

  /** puts the EPOS in homing mode */
  canmsg_t set_homing_mode_;

  /** sets the homing method to current mode positive speed */
  canmsg_t set_homing_method_positive_;

  /** sets the homing method to current mode positive speed */
  canmsg_t set_homing_method_negative_;

  /** puts the EPOS in start homing state */
  canmsg_t start_homing_;

  /** Resets the fault condition on the EPOS */
  canmsg_t clear_faults_;

  /** Function that defines a message with the right amount of quartercircles */
  canmsg_t set_position_mode_;
  
  canmsg_t start_absolute_position_;
  
  canmsg_t move_message;
  
  //Object to store all the data in, this is shared through all the object. 
  DataStore * const m_FtoW_data;

  int NODE_ID;
  int write_id_;   // the CAN adress of this controller, 1 or 2 or 4
  int read_id_;

  CANbus * const canbus;

  UI::ADAM * const adam_6017;
 
  bool moving_allowed;
  
  std::ofstream file_;
  
};
}
}

#endif	// DAAN_TEST1_MAXON_H
