/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "m_canbus.h"

using namespace top_level::canbus;

CANbusWrapper::CANbusWrapper(char device_name[20], unsigned int baudrate, unsigned int message_length, bool blocking) {
  fprintf(stderr, "CANbus Initialized with name /dev/%s\n",device_name);
}

int CANbusWrapper::write(unsigned long msg_id, unsigned char* msg_data, int msg_length){
  fprintf(stderr, "Written: %s to %i\n", msg_data, (int)msg_id);
}

int CANbusWrapper::read_all(std::vector<canmsg_t> *data){
  
  for(int i = 0; i<32; i++){
    data->push_back(canmsg_t());
    (*data)[i].data[0] = 'a';
    (*data)[i].data[1] = '8';
    (*data)[i].data[2] = '9';
    (*data)[i].id = 52 + i;
    (*data)[i].length = 3;
  }
  (*data)[2].data[2] = 'g';

  /*
  for(int i = 0; i<5; i++){
    (data+i)->data[0] = 13;
    (data+i)->data[1] = 'b';
    (data+i)->data[2] = 'c';
    (data+i)->length = 3;
    (data+i)->id = 120 + 5*i;
  }
for(int i = 5; i<10; i++){
    (data+i)->data[0] = 'a';
    (data+i)->data[1] = 'b';
    (data+i)->length = 2;
    (data+i)->id = 120 + 5*i;
  }
   * */
  return data->size();
}

int CANbusWrapper::parse_data(std::vector<canmsg_t>* messages){
  for(canmsg_t i : *messages){
    switch (i.id){
      case 52 :
        printf("Found a 52 with data: %s\n", i.data);
        break;
        
      case 54:
        printf("Found a 54 with data: %s\n", i.data);
        break;
        
      case 64:
        printf("Found a 62 with data: %s\n", i.data);
        break;
      
    }
  }
}



