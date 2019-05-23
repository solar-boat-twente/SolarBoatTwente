/* 
 * File:   PID_caller.h
 * Author: arjan
 *
 * Created on 11 maart 2019, 12:23
 */

#ifndef PID_CALLER_H
#define	PID_CALLER_H
#include "pid.h"
#include "DataStore.h"
#include "structures.h"
using namespace control;
using namespace MIO;
using namespace structures;
class PID_caller {
public:
    //PID(dt, Kp, Kd, Ki, Fc)
    struct pid_roll{
        double dt;
        double Kp;
        double Kd;
        double Ki;
        double Fc;
    };
    DataStore* m_complementary_data;
    DataStore* m_PID_data;
    void Setting_PID();
    PID_caller();
    PID_caller(const PID_caller& orig);
    virtual ~PID_caller();
    void PID_in();
    UserInput::SteeringInput * p =new UserInput::SteeringInput;
private:
};

#endif	/* PID_CALLER_H */

