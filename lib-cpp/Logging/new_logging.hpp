/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   new_logging.hpp
 * Author: Sander Oosterveld
 *
 * Created on May 30, 2019, 9:28 PM
 */

#ifndef NEW_LOGGING_HPP
#define NEW_LOGGING_HPP

class NewLogger{ 
  public:
      NewLogger(const std::string PathToConfig);
      NewLogger(const Logger& orig);
      virtual ~NewLogger();

      void write_values(const std::string var1, const std::string var2);
      std::string write_struct_user_power(const MIO::structures::PowerInput *power_input_ptr,const MIO::structures::PowerOutput *power_output_ptr, const MIO::structures::UserInput *user_input_ptr);
      std::string write_struct_control_data(const MIO::structures::ControlData *control_data_ptr);
      std::string write_struct_telemetry_input(const MIO::structures::TelemetryInput *telemetry_input_ptr);
  private:

};


#endif /* NEW_LOGGING_HPP */

