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

using namespace std;

namespace MIO{
namespace xsens{
class Sensor {

public:
    
    Sensor();   // Voor testen met data in een file
    virtual ~Sensor();


    Sensor(DataStore * xsens_state_data, DataStore * raw_data, Serial * serial = NULL )
        : xsens_state_data_(xsens_state_data), raw_data_(raw_data) {
      sensor_value = new DataStore::sensor_struct;
    };
    /**
     * Get the data from the xsens and write it to the 
     */
    void get_data();
    bool data_op();

private:
    // Alleen voor testen.
    bool data_is_op;
    DataStore * const raw_data_;
    DataStore *xsens_state_data_;
    DataStore::sensor_struct * sensor_value;

};
}
}
#endif // !defined(EA_E78C10D6_0F22_4073_B554_37CE1F63D0FB__INCLUDED_)