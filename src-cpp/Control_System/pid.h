/* 
 * File:   pid.h
 * Author: User
 *
 * Created on 27 februari 2019, 10:28
 */

#ifndef _PID_H_
#define _PID_H_
#include "DataStore.h"
#include "structures.h"

namespace control{
using namespace MIO;
using namespace structures;
class PIDImpl
{
    public:
        PIDImpl( float dt, float Kp, float Kd, float Ki, float Fc );
        ~PIDImpl();
        float calculate( float setpoint, float pv );
    private:
        float _dt;
        float _max;
        float _min;
        float _Kp;
        float _Kd;
        float _Ki;
        float _Fc;
        float _pre_error;
        float _integral;
};

class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        void PIDRoll();
        void PIDPitch();
        void PIDHeight();

        // Returns the manipulated variable given a setpoint and current process value
        float calculate( float setpoint, float pv );
        ~PID();
    UserInput::ControlUserInput * k =new UserInput::ControlUserInput;
    private:
        PIDImpl *pimpl;
};
}
#endif
