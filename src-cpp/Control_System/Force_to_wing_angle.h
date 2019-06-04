/* 
 * File:   Force_to_wing_angle.h
 * Author: User
 *
 * Created on 28 februari 2019, 10:05
 */

#ifndef FORCE_TO_WING_ANGLE_H
#define	FORCE_TO_WING_ANGLE_H
#include "DataStore.h"
namespace control{

class ForceToWingAngle {
public:

    ForceToWingAngle();
    ForceToWingAngle(const ForceToWingAngle& orig);
    DataStore *m_PID_data;
    DataStore *m_FtoW_data;
    DataStore *m_complementary_data;
    DataStore *m_xsens_state_data;
    virtual ~ForceToWingAngle();
    void MMA();    
private:

};   
}
#endif
