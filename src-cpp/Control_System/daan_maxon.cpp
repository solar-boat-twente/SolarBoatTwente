#include "Daan_Test1_maxon.h"
#include "../../lib-cpp/Canbus/canbus.h"
#include "../../lib-cpp/ADAM/ADAM.hpp"
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
using namespace std;
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
void EPOS::Homing(){  //foo is een teller, als can.write lukt komt er een 1 uit en als het faalt een 0. Uiteindelijk moet foo in dit geval dus 5 zijn.  
    canmsg_t m_Clear_Fault;
    m_Clear_Fault.id = 0x600+NODE_ID;
    m_Clear_Fault.length = 8;
    memcpy(&m_Clear_Fault.data[0],(char*)Clear_Fault_Data, 8);
    sanders_can->write_can(&m_Clear_Fault);
   std::this_thread::sleep_for(std::chrono::milliseconds(1500));
   // pc.printf("Waarde van foo is: %i \r\n", foo);
    canmsg_t m_Homing_Mode; 
    m_Homing_Mode.id = 0x600+NODE_ID;
    m_Homing_Mode.length = 8;
    memcpy(&m_Homing_Mode.data[0],(char*)Homing_Mode_Data, 8);
    sanders_can->write_can(&m_Homing_Mode);        
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
   // pc.printf("Waarde van foo is: %i \r\n", foo);
    canmsg_t m_Homing_Mode_Positive; 
    m_Homing_Mode_Positive.id = 0x600+NODE_ID;
    m_Homing_Mode_Positive.length = 8;
    memcpy(&m_Homing_Mode_Positive.data[0],(char*)Homing_Method_Data_Positive, 8);    
    sanders_can->write_can(&m_Homing_Mode_Positive);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
   // pc.printf("Waarde van foo is: %i \r\n", foo);
    canmsg_t m_Shutdown; 
    m_Shutdown.id = 0x600+NODE_ID;
    m_Shutdown.length = 8;
    memcpy(&m_Shutdown.data[0],(char*)Shutdown_Data, 8);    
    sanders_can->write_can(&m_Shutdown);      
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
   // pc.printf("Waarde van foo is: %i \r\n", foo);
    canmsg_t m_SwitchOnAndEnable; 
    m_SwitchOnAndEnable.id = 0x600+NODE_ID;
    m_SwitchOnAndEnable.length = 8;
    memcpy(&m_SwitchOnAndEnable.data[0],(char*)Switch_On_And_Enable_Data, 8);  
    sanders_can->write_can(&m_SwitchOnAndEnable);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
   // pc.printf("Waarde van foo is: %i \r\n", foo);
    canmsg_t m_StartHoming; 
    m_StartHoming.id = 0x600+NODE_ID;
    m_StartHoming.length = 8;
    memcpy(&m_StartHoming.data[0],(char*)Start_Homing_Data, 8);
    sanders_can->write_can(&m_StartHoming);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
}
void EPOS::HomingCheck(){
    
    int k=0;
    while(k<100){
    
    canmsg_t m_StatusWord;
    m_StatusWord.id = 0x600+NODE_ID;
    m_StatusWord.length = 4;
    memcpy(&m_StatusWord.data[0],(char*)Status_Word_Data, 4);
    sanders_can->write_can(&m_StatusWord);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    canmsg_t msg;
    sanders_can->read_can(0x580+NODE_ID, &msg);
    
            if (msg.data[5] == 0x95){ // case home found
                //#ifdef EPOS_DEBUG
                printf("Homeing NODE(%i) Succesfull!! \r\n",NODE_ID);
                moving_allowed = 1;
                //#endif
                //EPOS_HOME[NODE_ID-1]=1;
                return;
            }
            else if ( !(msg.data[4] == 0x37)){ // case homing failed
                //#ifdef EPOS_DEBUG
                printf("Homeing NODE(%i) FAILED!! \r\n",NODE_ID);
                moving_allowed=0;
                //#endif
                //EPOS_HOME[NODE_ID-1]=0;
                //return;
            }
    
     k=k+1;
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));  
    
} 
}
    
/* -----------------------------------------------------------------------------
In this boolean funtion, the position is set and the axis is moving to the new 
absolute position with the maximum acceleration and maximum velocity without a 
particular traject. 
With tel we built in a test to see if all the can messages are written on the 
CAN line. 
----------------------------------------------------------------------------- */

void EPOS::StartPositionMode(){
    //int tel=0;
    if (moving_allowed=1){
        
    canmsg_t m_Clear_Fault;
    m_Clear_Fault.id = 0x600+NODE_ID;
    m_Clear_Fault.length = 8;
    memcpy(&m_Clear_Fault.data[0],(char*)Clear_Fault_Data, 8);
    sanders_can->write_can(&m_Clear_Fault);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //pc.printf("Waarde van tel is: %i \r\n", tel);
    canmsg_t m_PositionMode;
    m_PositionMode.id = 0x600+NODE_ID;
    m_PositionMode.length = 8;
    memcpy(&m_PositionMode.data[0],(char*)Position_Mode_Data, 8);
    sanders_can->write_can(&m_PositionMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //pc.printf("Waarde van tel is: %i \r\n", tel);
    canmsg_t m_Shutdown; 
    m_Shutdown.id = 0x600+NODE_ID;
    m_Shutdown.length = 8;
    memcpy(&m_Shutdown.data[0],(char*)Shutdown_Data, 8);  
    sanders_can->write_can(&m_Shutdown);      
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //pc.printf("Waarde van tel is: %i \r\n", tel);
    canmsg_t m_SwitchOnAndEnable; 
    m_SwitchOnAndEnable.id = 0x600+NODE_ID;
    m_SwitchOnAndEnable.length = 8;
    memcpy(&m_SwitchOnAndEnable.data[0],(char*)Switch_On_And_Enable_Data, 8); 
    sanders_can->write_can(&m_SwitchOnAndEnable);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    else{
        printf("Maxon %i not homed", NODE_ID);
    }
    //pc.printf("Waarde van tel is: %i \r\n", tel);
           
    //can.write(GoToPosition(quartercircles));                                             //send new position to the controllers
    //Thread::wait(100);    
}  //end of StartPositionMode 

void EPOS::Move(){
    DataStore::AngleWings alphas = m_FtoW_data->GetWingData();
    std::cout<<"Result from 5: "<<adam_6->read_counter(5)<<endl;
    
    position_button = adam_6->read_counter(5);
      //366889 qc bij 1 rad hoekverdraaiing
    //float quartercircles= (-366889/alphas.Wing_right);
    float user;
    float user_wing;
    user = position_button/65536;
            if (user<0.5){
                user_wing= -1*user*1000000;
            } else {
                user_wing=user*1000000;
            }
    if (NODE_ID==4){
      //2000000qc bij 16 graden, dus 125000 bij 1 graad
        quartercircles=quartercircles + ((alphas.Wing_back*180/3.1415)*125000)+user_wing;   //absolute positie en start direct
        cout << "NODE = 4 \r\n" << endl;
            if (quartercircles < -800000)
            quartercircles = -800000;
            else if (quartercircles > 1500000)
            quartercircles = 1500000;
        cout << "positie achter =  \r\n"<<quartercircles ;
    }
    else if(NODE_ID==1) {
        //366889 qc bij 1 rad hoekverdraaiing
        //10000 qc bij 1 graad
        quartercircles=quartercircles + ((alphas.Wing_left*180/3.1415)*10000)+user_wing;
        cout << "NODE=1 \r\n" << endl;
            if (quartercircles < -100000)
            quartercircles = -100000;
            else if (quartercircles > 110000)
            quartercircles = 110000;
        cout << "alphas_links is: " << alphas.Wing_left << "\r\n"; 
        cout << "positie links is "<<quartercircles<<"\r\n";
        }
    else {
          //366889 qc bij 1 rad hoekverdraaiing
        quartercircles=quartercircles + ((alphas.Wing_right*180/3.1415)*10000)+user_wing;
        cout << "NODE = 2 \r\n" << endl;
        if (quartercircles < -100000)
            quartercircles = -100000;
        else if (quartercircles > 110000)
                quartercircles = 110000;
        cout << "positie links is "<<quartercircles << "\r\n";
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
    
    union {
        int integer;
        unsigned char byte[4];
    } QC;
    
    QC.integer = quartercircles;
    
    unsigned char Move_Data[8];
    Move_Data[0] = 0x23;
    Move_Data[1] = 0x7A;  //0x62;
    Move_Data[2] = 0x60;  //0x20;
    Move_Data[3] = 0x00;
    Move_Data[4] = QC.byte[0];
    Move_Data[5] = QC.byte[1];
    Move_Data[6] = QC.byte[2];
    Move_Data[7] = QC.byte[3];  
    canmsg_t m_Move;
    m_Move.id = 0x600+NODE_ID;
    m_Move.length = 8;
    memcpy(&m_Move.data[0],(char*) Move_Data, 8);
    sanders_can->write_can(&m_Move);
    
    canmsg_t m_Start_Absolute_Pos;
    m_Start_Absolute_Pos.id = 0x600+NODE_ID;
    m_Start_Absolute_Pos.length = 8;
    memcpy(&m_Start_Absolute_Pos.data[0],(char*) Start_Absolute_Pos, 8);
    sanders_can->write_can(&m_Start_Absolute_Pos);
}
        
              
       