#ifndef DAAN_TEST1_MAXON_H
#define DAAN_TEST1_MAXON_H

#include "DataStore.h"
#include "../../lib-cpp/Canbus/canbus.h"
using namespace MIO;

/* -----------------------------------------------------------------------------
EPOS commands that you need to home, you can find those in chapter 8.3 from the 
application node. 
----------------------------------------------------------------------------- */
const char Shutdown_Data[8] = {0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00};        //voorkomen dat je niet nog in een andere mode zit
    
const char Switch_On_And_Enable_Data[8] = {0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00}; //enable de motorcontroller (groene ledje constant aan)

const char Homing_Mode_Data[8] = {0x2F,0x60,0x60,0x00,0x06,0x00,0x00,0x00};     //aan de 0x6060-00 geef je aan dat je nu gaat homen wat de waarde 06 heeft in de data bytes
    
const char Homing_Method_Data_Positive[8] = {0x2F,0x98,0x60,0x00,0xFD,0x00,0x00,0x00};  //bij de index 0x6098-00 geef je aan welke homing method je gaat gebruiken: 0xFD = -3 = current treshold positive speed

const char Homing_Method_Data_Negative[8] = {0x2F,0x98,0x60,0x00,0xFC,0x00,0x00,0x00};  //bij de index 0x6098-00 geef je aan welke homing method je gaat gebruiken: 0xFC = -4 = current treshold negative speed
    
const char Start_Homing_Data[8] = {0x2B,0x40,0x60,0x00,0x1F,0x00,0x00,0x00}; //om te beginnen met homen geef je het controlword de waarde 0x001F om het homen te starten 

const char Status_Word_Data[4] = {0x40,0x41,0x60,0x00};  //0x40, want ccs is 2: je upload iets van de controller om te kijken of het gelukt is. 

const char Clear_Fault_Data[8] = {0x2B,0x40,0x60,0x00,0x80,0x00,0x00,0x00}; //Clear fault: 0x0080 versturen naar 0x6040-00    

/* -----------------------------------------------------------------------------
Additional EPOS commands that you need for going to a position, you can find 
those in chapter 8.7 from the application node. 
----------------------------------------------------------------------------- */
const char Position_Mode_Data[8] = {0x2F,0x60,0x60,0x00,0xFF,0x00,0x00,0x00};

/* -----------------------------------------------------------------------------
Sends a CAN Message to the controller
Returns 1 for succes. May be slow. a faster send message is build into Goto_Pos
@param COB_ID
@param Message length
@param Data string
----------------------------------------------------------------------------- */
canmsg_t create_CAN_msg(int COB_ID, int LENGTH, char * DATA);
    
//A struct in which msg_1 is saved
//struct EPOS_MESSAGES {
//    CANMessage msg_1;
//};

//CANMessage Latest_EPOS_msg(int NODE_ID);   
 
// The EPOS class
class EPOS{       
        
    public:
    
    //verbinding met main function
    DataStore *m_FtoW_data;
    
    int NODE_ID;   // the CAN adress of this controller
    CANbus * sanders_can;
    EPOS(int node_id){
        NODE_ID=node_id;
        };
    EPOS(CANbus * can, int node_id){
        NODE_ID=node_id;
        sanders_can=can;
        };
    int moving_allowed;
    float quartercircles = 0;
    /** puts the EPOS in shutdown state */
    canmsg_t Shutdown();
    
    /** puts the EPOS in Switchon state */
    canmsg_t SwitchOnAndEnable();
     
    /** Requests a statusword from the EPOS */
    canmsg_t StatusWord();
    
    /** puts the EPOS in homing mode */
    canmsg_t HomingMode();
    
    /** sets the homing method to current mode positive speed */
    canmsg_t HomingMethodPositive();
    
    /** sets the homing method to current mode positive speed */
    canmsg_t HomingMethodNegative();
    
    /** puts the EPOS in start homing state */
    canmsg_t StartHoming();
     
    /** Resets the fault condition on the EPOS */
    canmsg_t ClearFault();
    
    /** Function that defines a message with the right amount of quartercircles */
    //char* GoToPosition(float quartercircles);
    
    /** Function that defines a message with the right amount of quartercircles */
    canmsg_t PositionMode();

    /** Homes the EPOS with Current mode */
    void Homing();
    
    void HomingCheck();
    /** With this function the axis is set in the position mode */
    void StartPositionMode();
    
    /** With this function the axis is going to move */
    void Move();
};

#endif	// DAAN_TEST1_MAXON_H

