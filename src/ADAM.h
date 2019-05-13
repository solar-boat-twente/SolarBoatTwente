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



//TODO(Kneefel): Toevoegen van functie en class comments dit kun je het 
// makkelijkst doen door /** (twee sterren) en dan enter te doen op de lijn
// voor de functie, hierdoor worden er al een paar dingen ingevoerd, zie
// voorbeeld bij 'read_port'. Deze hoef je dan alleen maar in te vullen.
// class comments zijn iets lastiger daar beschrijf je eerst het doel van
// de klasse en daarna een voorbeeld hoe de class gebruikt kan worden

#ifndef ADAM_H
#define ADAM_H


class ADAM{
    
public:
    
 //TODO(Kneefel): Kun je hier niet ook port invullen, ik zou de eerste niet 
 // 'port' maar 'ip' noemen. Dan noem je de ander 'int port'
 // Ipv 'char*' zou ik char * const gebruiken --> de pointer moet immers constant
 // blijven.
    ADAM(char* port[]);
    
    /**
     * 
     * @param pin_number
     * @return 
     */
    bool read_port(int pin_number);
    
    //TODO (Kneefel): Ik zou port_state ipv port_value doen
    bool set_port(bool port_value, int port_number);
    
    //TODO(Kneefel): Deze moeten denk ik private zijn dus dan kun je ze ook zo
    // defineren dmv 'private:' 
    // Daarnaast zou ik rc geen field (class variabele) maken
    int rc;
    
    bool port_status[8];
    
    modbus_t *ctx;
    
    //TODO (kneefel): Misschien is een functie die de port_status stuurt ook wel nuttig.
    // je kunt in c++ niet direct een array returnen je moet dit doen dmv een buffer waar je het in gaat schrijven.
    // of een pointer naar het eerste element van de array.
};

#endif /* ADAM_H */

