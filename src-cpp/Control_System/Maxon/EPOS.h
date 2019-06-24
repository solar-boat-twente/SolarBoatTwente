#ifndef DAAN_TEST1_MAXON_H
#define DAAN_TEST1_MAXON_H

#include "../DataStore.h"
#include "../../../lib-cpp/Canbus/canbus.h"
#include "../../../lib-cpp/ADAM/ADAM.hpp"

#include "string"
#include <fstream>

namespace MIO{
namespace control{

constexpr int kPositionButtonQuartercountMultiplier = 3000000;//2000000;//2200000;

constexpr int MIN_ANGLE_LEFT = -100000;
constexpr int MAX_ANGLE_LEFT = 125000;

constexpr int MIN_ANGLE_RIGHT = -100000;
constexpr int MAX_ANGLE_RIGHT = 125000;

constexpr int MIN_ANGLE_BACK = 0;//-2000000;
constexpr int MAX_ANGLE_BACK = 2950000;//1100000;

constexpr int kQuartercountMultiplier = 10000;

constexpr uint16_t kStandardHomingWaitTime = 1500;
constexpr uint16_t kStandardPositionModeWaitTime = 1000;
constexpr uint16_t kStandardCheckHomingWaitTime = 1000;

/* -----------------------------------------------------------------------------
EPOS commands that you need to home, you can find those in chapter 8.3 from the 
application node. 
----------------------------------------------------------------------------- */
constexpr uint8_t kEposShutdownMessage[8] = {0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};        //voorkomen dat je niet nog in een andere mode zit
    
constexpr uint8_t kEposSwitchOnMessage[8] = {0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00}; //enable de motorcontroller (groene ledje constant aan)

constexpr uint8_t kEposHomingModeMessage[8] = {0x2f, 0x60, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};     //aan de 0x6060-00 geef je aan dat je nu gaat homen wat de waarde 06 heeft in de data bytes
   
constexpr uint8_t kEposHomingPositiveMessage[8] = {0x2F,0x98,0x60,0x00,0xFD,0x00,0x00,0x00};  //bij de index 0x6098-00 geef je aan welke homing method je gaat gebruiken: 0xFD = -3 = current treshold positive speed

constexpr uint8_t kEposHomingNegativeMessage[8] = {0x2F,0x98,0x60,0x00,0xFC,0x00,0x00,0x00};  //bij de index 0x6098-00 geef je aan welke homing method je gaat gebruiken: 0xFC = -4 = current treshold negative speed
    
constexpr uint8_t kEposStartHomingMessage[8] = {0x2B,0x40,0x60,0x00,0x1F,0x00,0x00,0x00}; //om te beginnen met homen geef je het controlword de waarde 0x001F om het homen te starten 

constexpr uint8_t kEposGetStatusMessage[4] = {0x40,0x41,0x60,0x00};  //0x40, want ccs is 2: je upload iets van de controller om te kijken of het gelukt is. 

constexpr uint8_t kEposClearFaultMessage[8] = {0x2B,0x40,0x60,0x00,0x80,0x00,0x00,0x00}; //Clear fault: 0x0080 versturen naar 0x6040-00    

constexpr uint8_t kEposMoveMessage[8] = {0x23, 0x7a, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
/* -----------------------------------------------------------------------------
Additional EPOS commands that you need for going to a position, you can find 
those in chapter 8.7 from the application node. 
----------------------------------------------------------------------------- */
constexpr uint8_t kEposSetPositionModeMessage[8] = {0x2F,0x60,0x60,0x00,0x01,0x00,0x00,0x00};

constexpr uint8_t kEposAbsolutePositionMessage[8] = {0x2B,0x40,0x60,0x00,0x3F,0x00,0x00,0x00};


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
  EPOS(CANbus * can, UI::ADAM * adam, int node_id, DataStore * control_data, std::time_t now);
  
  EPOS();
  
  ~EPOS();
   
  void Homing();
  /** Homes the EPOS with Current mode */
  void start_homing(bool home_positive = true, short int delay = kStandardHomingWaitTime);

  bool check_homing(int delay = kStandardCheckHomingWaitTime);
  
  /** With this function the axis is set in the position mode */
  void start_position_mode(short int delay = kStandardPositionModeWaitTime);

  /** With this function the axis is going to move */
  void move();
    
 private:
  
  canmsg_t create_CAN_msg(int id, int length, const uint8_t data[]);

  float angle2quartercounts_(float radians, const int min, const int max);
  
  float quartercounts2quartercounts_(float quartercounts, const int min, const int max);
  
  float rad2deg_(float radians);
  
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
  DataStore * const control_data_;

  int node_id_;
  int write_id_;   // the CAN adress of this controller, 1 or 2 or 4
  int read_id_;

  CANbus * const canbus_;

  UI::ADAM * const adam_6017;
 
  bool moving_allowed;
  
  std::ofstream file_;
  
  std::string file_name_;
  
};
}
}

#endif	// DAAN_TEST1_MAXON_H
