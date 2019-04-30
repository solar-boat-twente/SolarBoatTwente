/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   logging.h
 * Author: ronald
 *
 * Created on 05 April 2019, 08:51
 */

#ifndef LOGGING_H
#define LOGGING_H
#include "easylogging++.h"
#include <string>
#include <array>
#include "structures.h"
#include "convert_power.h"

class Logger { 
public:
    Logger(const std::string PathToConfig);
    Logger(const Logger& orig);
    virtual ~Logger();

    void write_values(const std::string var1, const std::string var2);
    void write_struct_user_power_to_log(const structures::PowerInput *power_input_ptr,const structures::PowerOutput *power_output_ptr, const structures::UserInput *user_input_ptr);
    void write_struct_control_data_to_log(const structures::ControlData *control_data_ptr);
    void write_struct_telemetry_input_to_log(const structures::TelemetryInput *telemetry_input_ptr);
private:

};

#endif /* LOGGING_H */

