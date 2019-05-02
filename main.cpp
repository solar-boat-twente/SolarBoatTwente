/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: Sander Oosterveld
 *
 * Created on April 30, 2019, 9:37 AM
 */

#include <cstdlib>
#include <stdlib.h>
#include <string>
#include "solarboattwente.h"
#include "lib-cpp/Logging/thread.h"
#include "lib-cpp/Logging/logging.h"
//#include "lib-cpp/Logging/easylogging++.h"



using namespace std;

/*
 * 
 */
int gen_rand()
{
    return std::rand()  % 100;
}


int main(int argc, const char** argv) {
   Thread Threads; 
   std::srand(std::time(nullptr));
   Threads.CreateThreads();
   
   MIO::structures::PowerInput *power_input_ptr, power_input;
   power_input_ptr = &power_input;
   MIO::structures::PowerOutput *power_output_ptr, power_output;
   power_output_ptr = &power_output;
   MIO::structures::UserInput *user_input_ptr, user_input;
   user_input_ptr = &user_input;
   MIO::structures::ControlData *control_data_ptr, control_data;
   control_data_ptr = &control_data;
   MIO::structures::TelemetryInput *telem_input_user, telem_input;
   telem_input_user = &telem_input;
   
   // creating logger instance with path to logger
   Logger control_data_logger("/root/SolarBoat2019/SolarBoatTwente/config/control_data.conf");
   
   while(1)
   {

        control_data_ptr->xsens.raw_pitch =gen_rand(); 
        control_data_ptr->xsens.raw_roll  =gen_rand();
        control_data_ptr->xsens.filtered_pitch =gen_rand(); 
        control_data_ptr->xsens.filtered_roll =gen_rand();
        control_data_ptr->xsens.raw_z_acceleration =gen_rand(); 
        control_data_ptr->vlotters.angle_left =gen_rand();
        control_data_ptr->vlotters.angle_right =gen_rand();
        control_data_ptr->computed.force_roll =gen_rand();
        control_data_ptr->computed.force_pitch =gen_rand(); 
        control_data_ptr->computed.force_height =gen_rand();
        control_data_ptr->computed.angle_left =gen_rand();
        control_data_ptr->computed.angle_right =gen_rand();
        control_data_ptr->computed.angle_back=gen_rand();
      
   
   
   
   
   //control_data_ptr->computed.angle_back = gen_rand();
   //power_input_ptr->battery.state_of_charge = 9.3;
   //user_power_logger.write_struct_user_power_to_log(power_input_ptr, power_output_ptr, user_input_ptr);
   //write the struct to the logfile
   //control_data_logger.write_struct_control_data(control_data_ptr);
   Threads.writeControlData(control_data_ptr, 0 ,1);
   //Threads.writeUserPower(power_input_ptr, power_output_ptr, user_input_ptr, 1, 0);
   //Threads.writeTelemInput(telem_input_user, 0, 1);
   //usleep(20000); 
   }
   return 0;
}