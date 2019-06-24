/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

 
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <cstdlib>
#include <errno.h>
#include <stdint.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "ADAM.hpp"
#include "../../lib-cpp/Debugging/easy_debugging.hpp"

using namespace std;
//TODO(Kneefel): Zorg dat alle indents kloppen met de google style guide, zoek 
//TODO(Kneefel): Ook hier moeten nog comments bij.

using namespace MIO;
using namespace UI;

ADAM::ADAM(const char* ip, int port_number, bool debug) {
  
  //first open the modbus port:
  ctx = modbus_new_tcp(ip, port_number);
  modbus_set_debug(ctx, debug);
  if(modbus_connect(ctx)<0){
    M_ERR<<"UNABLE TO CONNECT TO MODBUS: "<<modbus_strerror(errno);
    modbus_free(ctx);
  } else {
    M_OK<<"SUCCESSFULLY OPENED MODBUS CONNECTION";
  }
}

int ADAM::write_port(int port_number, bool state) {
  return modbus_write_bit(ctx, port_number, state);
}

int ADAM::read_counter(int port_number) {
  uint16_t *tab_rp_registers;
  tab_rp_registers = (uint16_t *) malloc(sizeof(uint16_t));
  
  modbus_read_registers(ctx, port_number, 1, tab_rp_registers);
  return *tab_rp_registers; 
}
 

