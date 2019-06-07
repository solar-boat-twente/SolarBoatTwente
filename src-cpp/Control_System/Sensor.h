///////////////////////////////////////////////////////////
//  sensor.h
//  Implementation of the Class sensor
//  Created on:      20-Feb-2019 11:56:45
//  Original author: wiegmink
///////////////////////////////////////////////////////////

#if !defined(EA_E78C10D6_0F22_4073_B554_37CE1F63D0FB__INCLUDED_)
#define EA_E78C10D6_0F22_4073_B554_37CE1F63D0FB__INCLUDED_

#include <iostream>
#include <fstream>
#include "DataStore.h"
#include "Xsens.h"
using namespace MIO;
using namespace std;
using namespace xsens;
class Sensor
{

public:
    int j=0; //gebruiken om te tellen. Na iedere seconde een waarde van 80 (systeem runt op 80Hz).
    
    Sensor();   // Voor testen met data in een file
    virtual ~Sensor();
    // Netter om geen public variabelen te hebben, maar alleen private.
    DataStore *m_ruwe_state_data;
    DataStore *m_xsens_state_data;
    Xsens * sens; 
    Serial * serie;
    Sensor(Xsens * m_sens,Serial * m_serial ){
        sens=m_sens;
        serie=m_serial;
    }
    void get_data();
    bool data_op();

private:
    // Alleen voor testen.
    bool data_is_op;

};
#endif // !defined(EA_E78C10D6_0F22_4073_B554_37CE1F63D0FB__INCLUDED_)