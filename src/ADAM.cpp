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
#include <modbus.h>
#include <stdint.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "ADAM.h"
#include "easy_debugging.hpp"

using namespace std;


ADAM::ADAM(char* port[]) {
    
    ctx = modbus_new_tcp("169.254.181.204", 502);
    modbus_set_debug(ctx, TRUE);
    
    if (modbus_connect(ctx) == -1)
    {
        fprintf(stderr, "Connection failed: %s\n",
        modbus_strerror(errno));
        modbus_free(ctx);  
    }
}


bool ADAM::read_port(int port_number){
    
    uint8_t tab_rp_bits[8];
    uint8_t value;
    int nb_points;
    int rc;
    int i;
    
    rc = modbus_read_input_bits(ctx, 0x00,
                                0x08, tab_rp_bits);
    i = 0;
    nb_points = 0x08;
    while (nb_points > 0)
    {
        int nb_bits = (nb_points > 8) ? 8 : nb_points;
        value = modbus_get_byte_from_bits(tab_rp_bits, i*8, nb_bits);
        //value = modbus_get_byte_from_bits(tab_rp_bits, i, 1);
        //printf("%i ", value);
        nb_points -= nb_bits;
        i++;    
    }
    
    for (int i = 0; i <= 7; i++)
    {
        if (((value>>i) % 2) == 0)
            port_status[i] = false;
        else
            port_status[i] = true; 
    }
    
    
    return port_status[port_number];
}

bool ADAM::set_port(bool port_value, int port_number){
    
    uint8_t port_change;
    
    switch(port_number){
        case '0' :
            port_change = 0x00;
        case '1' :
            port_change = 0x01;
        case '2' :
            port_change = 0x02;
        case '3' :
            port_change = 0x03;
        case '4' :
            port_change = 0x04;
        case '5' :
            port_change = 0x05;
        case '6' :
            port_change = 0x06;
        case '7' :
            port_change = 0x07;
        case '8' :
            port_change = 0x08;
        case '9' :
            port_change = 0x09;
        case 'A' :
            port_change = 0x0A;
        case 'B' :
            port_change = 0x0B;
    }
    
    rc = modbus_write_bit(ctx, port_change, port_value);
    
    if (port_value == 1)
        port_status[port_number] = true;
    else
        port_status[port_number] = false;
}


