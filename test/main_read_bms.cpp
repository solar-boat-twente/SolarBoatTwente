/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * @File:   main.cpp
 * @Author: Sander Oosterveld
 *
 * Created on April 9, 2019, 7:22 PM
 */

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

#include "../include/easy_debugging.hpp"
#include "../src/Wrappers/Serial.h"
#include "../src/Wrappers/canbus.h"
#include "../src/structures.h"
#include "../src/Battery_Magagement_System/BMS.h"
#include "../src/Steering_Wheel/Control_Wheel.hpp"
#include "../src/Genasun_Watt_Sensor/MPPT.h"

using namespace std;
using namespace MIO;


int MAX = 500;

int main(){
  Serial * m_serial = new Serial("/dev/ttyUSB0");
  m_serial->write_byte(51);
  m_serial->write_byte(10);
  m_serial->write_byte(0);
  uint8_t buffer[2];
  m_serial->read_bytes(buffer, 2);
  
  
}


/*
int main(){
  CANbus * canbus = new CANbus("can1");
  MIO::PowerElectronics::MPPT * mppt = new PowerElectronics::MPPT(canbus);
  
  mppt->set_relay_from_number(true, 5);
  mppt->set_relay_from_number(true, 12);
  mppt->set_relay_from_number(true, 9);
  mppt->set_relay_from_number(true, 10);
  mppt->set_relay_from_number(false, 9);


}
*/


//
//int main(){ 
//  
//  Serial * const m_serial = new Serial("/dev/ttyUSB0");
//  uint8_t buffer[20];
//  
//  structures::UserInput * const user_input = new structures::UserInput;
//  
//  UI::ControlWheel * const control_wheel = new UI::ControlWheel(m_serial);
//  control_wheel->start_reading(user_input);
//  while(true){}
//  this_thread::sleep_for(chrono::milliseconds(500));
//  
//  
//  //m_serial->read_bytes(buffer, 20);
//  delete m_serial;
//}
//


//
//int main(){
//  CANbus * m_canbus = new CANbus("can0");
//  CANbus * receive_canbus = new CANbus("can1",1);
//  receive_canbus->open_can(O_RDONLY);
//  m_canbus->open_can();
//  
//  
//  receive_canbus->start(5);
//  canmsg_t m_tx[50];
//  char standard_can[8] = {3, 5, 8, 9 , 10, 11, 12, 16};
//  for(int i=0; i<50; i++){
//    m_tx[i].id = 500+i;
//    m_tx[i].length = 7;
//    memcpy(&m_tx[i].data[0],&standard_can[0], 7);
//  }
//  M_INFO<<"CONSTRUCTED 50 MESSAGES!";
//  for(int i = 0; i<50; i++){
//    m_canbus->write_can(&m_tx[i]);
//  }
//  this_thread::sleep_for(chrono::milliseconds(1000));
//  m_canbus->close_can();
//  canmsg_t message;
//
//  for(int i = 10; i<40; i++){
//    receive_canbus->read_can(500+i, &message);
//    print_canmsg(&message);
//  }
//
//  this_thread::sleep_for(chrono::milliseconds(5000));
//  receive_canbus->close_can();
//  
//   return 0;
// 
//};


//int main(){
//  Serial * m_serial = new Serial("/dev/ttyUSB0");
//  
//  char responses[5] = {1, 4, 8, 9 , 10};
//  cout<<"REPONSES[-1]:"<<responses[-1]<<"\n";
////  m_serial->read_bytes(responses, 4);
//
//  char temp[20];
//  char stop[2] = {'\r','\n'};
//  int message_length = m_serial->read_stop(temp, stop,2, 20);
//  M_INFO << "Response: ";
//  for(int i = 0; i<message_length; i++){
//    printf("%i ", temp[i]);
//  } 
//  printf("\n");
//}


//int main(){
//
//  int   USB = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
//  struct termios tty;
//  struct termios tty_old;
//  memset (&tty, 0, sizeof tty);
//
//  /* Error Handling */
//  if ( tcgetattr ( USB, &tty ) != 0 ) {
//     std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
//  }
//
//  /* Save old tty parameters */
//  tty_old = tty;
//
//  /* Set Baud Rate */
//  cfsetospeed (&tty, (speed_t)B9600);
//  cfsetispeed (&tty, (speed_t)B9600);
//
//  /* Setting other Port Stuff */
//  tty.c_cflag     &=  ~PARENB;            // Make 8n1
//  tty.c_cflag     &=  ~CSTOPB;
//  tty.c_cflag     &=  ~CSIZE;
//  tty.c_cflag     |=  CS8;
//
//  tty.c_cflag     &=  ~CRTSCTS;           // no flow control
//  tty.c_cc[VMIN]   =  1;                  // read doesn't block
//  tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
//  tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
//
// /* Make raw */
//  cfmakeraw(&tty);
//
// /* Flush Port, then applies attributes */
//  tcflush( USB, TCIFLUSH );
//  if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
//     std::cout << "Error " << errno << " from tcsetattr" << std::endl;
//  }
//
//  printf("Opened reader with file descirption %d\n", USB);
//  int n_written = write(USB, "3000\r\n", 6);
//  M_INFO<<"WRITTEN 1 TO FD: "<<n_written;
//
//  int n = 0, spot = 0;
//  char buf = '\0';
//
// /* Whole response*/
//  char response[20];
//  while(true){
//    spot = 0;
//    memset(response, '\0', sizeof response);
//    while(true){
//        n = read( USB, &buf, 1 );
//        sprintf( &response[spot], "%c", buf );
//        M_INFO<<"READ SOMETHING: "<<buf;
//        if((buf == '\n')&&response[spot-1]=='\r'){
//          break;
//        } 
//        if (n<0){
//          break;
//        }
//        spot+=1;
//    } 
//
//    if (n < 0) {
//        std::cout << "Error reading: " << strerror(errno) << std::endl;
//    }
//    else if (n == 0) {
//        std::cout << "Read nothing!" << std::endl;
//    }
//    else {
//        M_INFO << "Response: ";
//        for(int i = 0; i<spot-1; i++){
//          printf("%i ", response[i]);
//        } 
//        printf("\n");
//    }
//  }
//  return 0;
//  
//}

/*
int main(int argc, char** argv) {
//  char response1[] = {80, 97, 168, 0, 0, 178, 2, 5};
//  char response2[] = {60,30,1,0,1};
//  char response3[] = {11, 184, 15, 160, 19, 136, 23, 112};
  
  CANbus * const canbus = new CANbus("can0", 1);
  structures::PowerInput * const power_input = new structures::PowerInput;
  structures::PowerOutput * const power_output = new structures::PowerOutput;
  
  
  
//  canmsg_t *message1 = new canmsg_t;
//  canmsg_t *message2 = new canmsg_t;
//  canmsg_t *message3 = new canmsg_t;
//  canmsg_t *message4 = new canmsg_t;
//  message1->id = CANID_BMS_RESPONSE1;
//  message1->length = 8;
//  for(int i=0; i<message1->length;i++){
//    message1->data[i] = response1[i];
//  }
//  
//  message2->id = CANID_BMS_RESPONSE2;
//  message2->length = 5;
//  for(int i=0; i<message2->length; i++){
//    message2->data[i] = response2[i];
//  }
//      
//  message3->id = CANID_BMS_CELL_VOLTAGE_START;
//  message3->length = 8;
//  for(int i=0; i<message3->length;i++){
//    message3->data[i] = response3[i];
//  }
//  
//  *message4 = *message3;
//  
//  message4->id=CANID_BMS_CELL_VOLTAGE_START+1;
//  
//  canbus->add_message_(message1);
//  canbus->add_message_(message2);
//  canbus->add_message_(message3);
//  canbus->add_message_(message4);
 
  power_output->contractor_control = 1;
  canbus->start(10);
  PowerElectronics::BMS * const bms = new PowerElectronics::BMS(canbus);
  this_thread::sleep_for(chrono::milliseconds(2000));
  bms->start_reading(power_input);
  this_thread::sleep_for(chrono::milliseconds(3000));
  bms->start_writing(power_output);
  
  cout<<INFO("\n============================================= OUTPUT DATA ==========================================")<<endl;
  fprintf(stderr,INFO("soc: %f\tvoltage: %f\tcurrent: %f\terror: %i\t error_loc: %i\n"),
      power_input->battery.state_of_charge, power_input->battery.total_voltage, power_input->battery.total_current,
      power_input->battery.error_number, power_input->battery.error_location);
  fprintf(stderr, INFO("Max_temp: %i\tMin_temp: %i\tBalance: %i\tCont_ready: %i\tCont_stat: %i\n"),
          power_input->battery.max_temp, power_input->battery.min_temp, power_input->battery.balance_state,
          power_input->battery.contactor_ready, power_input->battery.contactor_status);
  for(int i = 0; i<12; i++){
    printf(INFO("Cell %i: %f\t"), i+1, power_input->battery.cel_voltages[i]);
    if(i%4==3){
      printf("\n");
    }
  }    
  cout<<INFO("====================================================================================================")<<endl;
  
  this_thread::sleep_for(chrono::seconds(5));
  bms->stop_reading();
  bms->stop_writing();
  canbus->stop();
  canbus->close_can();
  return 0;
}
*/ 


