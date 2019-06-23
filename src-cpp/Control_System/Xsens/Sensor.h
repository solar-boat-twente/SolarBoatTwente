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

#include "../DataStore.h" 
#include "Xsens.h"
#include "../Vlotter/Vlotter.hpp"

namespace MIO{
namespace xsens{

class Sensor {

public:
    
    Sensor();   // Voor testen met data in een file

    Sensor(DataStore * const control_data, control::Vlotter * const vlotter);

    virtual ~Sensor();

    // rule of 5:
    Sensor(const Sensor&);

    Sensor operator=(const Sensor&);

    Sensor(Sensor&&) = delete;

    Sensor& operator=(Sensor&&) = delete;


    /**
     * Get the data from the xsens and write it to the 
     */
    void update_data();

private:

    DataStore * const control_data_;
    control::Vlotter * const vlotter_;
    
    DataStore::sensor_struct * const sensor_data_;

};
}
}
#endif // !defined(EA_E78C10D6_0F22_4073_B554_37CE1F63D0FB__INCLUDED_)