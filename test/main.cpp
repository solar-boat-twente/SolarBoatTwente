/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: Sander Oosterveld
 *
 * Created on April 9, 2019, 7:22 PM
 */

#include <cstdlib>
#include <stdio.h>
#include <vector>
#include <iostream>
#include "../src/Wrappers/canbus.h"

using namespace std;
using namespace MIO;
/*
 * 
 */
int main(int argc, char** argv) {
  CANbus canbus("can0", 100);
  canmsg_t message;
  canmsg_t msg;
  message.data[0] = 'a';
  message.data[1] = 'a';
  message.data[2] = 'b';
  message.id = 12948;
 canbus.add_message_(&message);
  printf("Result of reading id 12948: %i\n", canbus.read(12948, &msg));
  printf("Result:\t\t Data: %s id: %i\n", msg.data,(int)msg.id);
  message.data[1] = 'c';
  canbus.add_message_(&message);
  
  printf("Result of reading id 12948: %i\n", canbus.read(12948, &msg));
  printf("Result:\t\t Data: %s id: %i\n", msg.data,(int)msg.id);
    printf("Result of reading id 12948: %i\n", canbus.read(12948, &msg));
  printf("Result:\t\t Data: %s id: %i\n", msg.data,(int)msg.id);
    printf("Result of reading id 12548: %i\n", canbus.read(12548, &msg));
  printf("Result:\t\t Data: %s id: %i\n", msg.data,(int)msg.id);
  message.data[0] = 'a';
  message.data[1] = 'b';
  message.data[2] = 'c';
  message.id = 12948;
  canbus.write(&message);
  
  
  
  return 0;
}

