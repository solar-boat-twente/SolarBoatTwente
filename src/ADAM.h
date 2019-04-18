/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   ADAM.h
 * Author: Thomas Kneefel
 *
 * Created on 15 april 2019, 15:53
 */

#ifndef ADAM_H
#define ADAM_H


class ADAM{
    
public:
    
    ADAM(char* port[]);
    
    bool read_port(int pin_number);
    
    bool set_port(bool port_value, int port_number);
    
    int rc;
    
    bool port_status[8];
    
    modbus_t *ctx;
};

#endif /* ADAM_H */

