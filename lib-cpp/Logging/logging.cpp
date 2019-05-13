#include "logging.h"

using namespace MIO;
using namespace structures;

Logger::Logger(const std::string PathToConfig)
{
    // constructor: variables initialzing 
    el::Configurations conf(PathToConfig);
    //el::Configurations conf("configuration.conf");
    // Reconfigure single logger
    el::Loggers::reconfigureLogger("default", conf);
    // Actually reconfigure all loggers instead
    //el::Loggers::reconfigureAllLoggers(conf);

}

Logger::Logger(const Logger& orig)
{
}

Logger::~Logger()
{
    // ~ deconstructor
}

INITIALIZE_EASYLOGGINGPP


void Logger::write_struct_user_power(const structures::PowerInput *power_input_ptr, const structures::PowerOutput *power_output_ptr, const structures::UserInput *user_input_ptr){
    //std::string testing_func;
    //testing_func = append_array_to_csv_format(power_input_ptr->battery.cel_voltages);
    LOG(INFO) << "," << power_input_ptr->battery.cel_voltages[0] << "," << power_input_ptr->battery.cel_voltages[1] << "," << power_input_ptr->battery.cel_voltages[2] << "," << power_input_ptr->battery.cel_voltages[3] 
           << "," << power_input_ptr->battery.cel_voltages[4] << "," << power_input_ptr->battery.cel_voltages[5] << "," << power_input_ptr->battery.cel_voltages[6] << "," << power_input_ptr->battery.cel_voltages[7] 
           << "," << power_input_ptr->battery.cel_voltages[8] << "," << power_input_ptr->battery.cel_voltages[9] << "," << power_input_ptr->battery.cel_voltages[10] << "," << power_input_ptr->battery.cel_voltages[11] 
           << ","<< power_input_ptr->battery.state_of_charge << "," << power_input_ptr->battery.error_number 
           << ","<< power_input_ptr->battery.error_location << "," << power_input_ptr->battery.max_temp<< ","  << power_input_ptr->battery.min_temp << "," << power_input_ptr->battery.balance_state
           << "," << power_input_ptr->battery.contactor_ready << "," << power_input_ptr->battery.contactor_status 
           << ","<< power_input_ptr->solar_panels.MPPT_power[0] << ","<< power_input_ptr->solar_panels.MPPT_power[1] << ","<< power_input_ptr->solar_panels.MPPT_power[2] 
           << ","<< power_input_ptr->solar_panels.MPPT_power[3] << ","<< power_input_ptr->solar_panels.MPPT_power[4] << ","<< power_input_ptr->solar_panels.MPPT_power[5] 
           << ","<< power_input_ptr->solar_panels.MPPT_power[6] << ","<< power_input_ptr->solar_panels.MPPT_power[7] << ","<< power_input_ptr->solar_panels.MPPT_power[8] 
           << ","<< power_input_ptr->solar_panels.MPPT_power[9] 
           << ","<< power_input_ptr->solar_panels.panel_power[0] << ","<< power_input_ptr->solar_panels.panel_power[1] << ","<< power_input_ptr->solar_panels.panel_power[2] 
           << ","<< power_input_ptr->solar_panels.panel_power[3] << ","<< power_input_ptr->solar_panels.panel_power[4] << ","<< power_input_ptr->solar_panels.panel_power[5] 
           << ","<< power_input_ptr->solar_panels.panel_power[6] << ","<< power_input_ptr->solar_panels.panel_power[7] << ","<< power_input_ptr->solar_panels.panel_power[8] 
           << ","<< power_input_ptr->solar_panels.panel_power[9] 
           << ","<< power_input_ptr->driver.motor_temp <<  "," << power_input_ptr->driver.driver_temp << ","<< power_input_ptr->driver.driver_output_power 
           << ","<< power_input_ptr->driver.motor_speed << ","<< power_input_ptr->driver.driver_voltage_input << "," << power_input_ptr->driver.driver_current_input
           << ","<< power_input_ptr->driver.driver_state 
           << ","<< power_output_ptr->solar_panel_states[0] << ","<< power_output_ptr->solar_panel_states[1]<< ","<< power_output_ptr->solar_panel_states[2]
           << ","<< power_output_ptr->solar_panel_states[3]<< ","<< power_output_ptr->solar_panel_states[4]<< ","<< power_output_ptr->solar_panel_states[5]
           << ","<< power_output_ptr->solar_panel_states[6]<< ","<< power_output_ptr->solar_panel_states[7]<< ","<< power_output_ptr->solar_panel_states[8]
           << ","<< power_output_ptr->solar_panel_states[9]
            
           << "," << power_output_ptr->throttle << "," << power_output_ptr->motor_state << "," << power_output_ptr->contractor_control 
           << ","<< power_output_ptr->balancing_control
           <<","<< user_input_ptr ->control.PID_roll <<","<< user_input_ptr ->control.PID_pitch <<","<< user_input_ptr ->control.PID_height 
           <<","<< user_input_ptr ->buttons.battery_on <<","<< user_input_ptr ->buttons.force_battery <<","<< user_input_ptr ->buttons.motor_on
           <<","<< user_input_ptr ->buttons.debug_on
           << ","<< user_input_ptr->steer.raw_throttle << "," << user_input_ptr->steer.fly_mode << "," << user_input_ptr->steer.reverse
            
           << ","<< power_input_ptr->battery.total_voltage << ","<< power_input_ptr->battery.total_current <<","<< power_input_ptr->driver.driver_on
            <<","<< power_input_ptr->driver.error_word <<","<< power_input_ptr->driver.low_priority_limiter << ","<< power_output_ptr->real_throttle;
}

void Logger::write_struct_control_data(const structures::ControlData *control_data_ptr){
   LOG(INFO) << "," << control_data_ptr->xsens.raw_pitch << "," << control_data_ptr->xsens.raw_roll << "," << control_data_ptr->xsens.filtered_pitch << ","
           << control_data_ptr->xsens.filtered_roll << "," << control_data_ptr->xsens.raw_z_acceleration << ","
           << control_data_ptr->vlotters.angle_left << "," << control_data_ptr->vlotters.angle_right << ","
           << control_data_ptr->computed.force_roll << "," << control_data_ptr->computed.force_pitch << "," << control_data_ptr->computed.force_height << ","
           << control_data_ptr->computed.angle_left << "," << control_data_ptr->computed.angle_right << "," << control_data_ptr->computed.angle_back
           << control_data_ptr->real_height << ","   << control_data_ptr->real_roll << "," 
            << control_data_ptr->xsens.speed << ","   << control_data_ptr->xsens.latitude << ","  << control_data_ptr->xsens.longitude << ","
            << control_data_ptr->current_P << "," << control_data_ptr->current_I << "," << control_data_ptr->current_D << "," << control_data_ptr->current_N;
}

void Logger::write_struct_telemetry_input(const structures::TelemetryInput *telemetry_input_ptr){
   LOG(INFO) << "," << telemetry_input_ptr->control.PID_height.P  << "," << telemetry_input_ptr->control.PID_height.I  << "," << telemetry_input_ptr->control.PID_height.D  << "," << telemetry_input_ptr->control.PID_height.N  << "," 
           << "," << telemetry_input_ptr->control.PID_roll.P  << "," << telemetry_input_ptr->control.PID_roll.I  << "," << telemetry_input_ptr->control.PID_roll.D  << "," << telemetry_input_ptr->control.PID_height.N  << "," 
           << "," << telemetry_input_ptr->control.PID_pitch.P  << "," << telemetry_input_ptr->control.PID_pitch.I  << "," << telemetry_input_ptr->control.PID_pitch.D  << "," << telemetry_input_ptr->control.PID_pitch.N  << "," 
           << telemetry_input_ptr->control.overwrite  << ","
           << telemetry_input_ptr->solar_panel_states[0]  << "," << telemetry_input_ptr->solar_panel_states[1]  << ","<< telemetry_input_ptr->solar_panel_states[2]  << ","
           << telemetry_input_ptr->solar_panel_states[3]  << ","<< telemetry_input_ptr->solar_panel_states[4]  << ","<< telemetry_input_ptr->solar_panel_states[5]  << ","
           << telemetry_input_ptr->solar_panel_states[6]  << ","<< telemetry_input_ptr->solar_panel_states[7]  << ","<< telemetry_input_ptr->solar_panel_states[8]  << ","
           << telemetry_input_ptr->solar_panel_states[9]  << ","
           << "," << telemetry_input_ptr->advised_speed;
}



