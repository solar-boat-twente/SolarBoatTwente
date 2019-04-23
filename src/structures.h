/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   library.h
 * Author: Sander Oosterveld
 *
 * Created on February 28, 2019, 9:13 AM
 */

#ifndef STRUCTURES_H
#define STRUCTURES_H

/*
 Declaring all the structures 
 */

/*
 * All the input structures and nested input structures for the power data.
 *
 *
 */
namespace MIO{
namespace structures {
//Namespace containing all the global structures

struct PowerInput {
  /*
   All the input gotten from the power electronics, this data is split as follows:
   * - battery (all the data from the battery)
   * - solarPanels (all the data from the solarPanels)
   * - driver (all the data from the motorController)
   */

  //The main structure for all of the power, this is subdivided into battery, solar and motor

  /*
   Nested structures used to order the PowerInput structure
   */
  struct BatteryInput {
    // Voltage of the battery back (V)
    float total_voltage;
    
    // Current drawn from the battery_pack (A)
    float total_current;
    
    //The cel_voltages in V
    float cel_voltages[12];
    
    //The state of charge number between 0 and 100%, 0.5% precise
    float state_of_charge;
    
    //The state of health: number between 0 and 100%, 0.5% precies
    float state_of_health;
    
    //The raw error_number from the BMS
    unsigned char error_number;
    
    //The cell number in which the error occured
    unsigned char error_location;
    
    //The maximum temperature in the BMS in degC
    short int max_temp;
    
    //The minimum temperature in the BMS in degC
    short int min_temp;
    
    //The balancing state of the BMS, False: not balancing, True: balancing
    bool balance_state; 
    
    //Wether the BMS thinks the contactor should be connected, True is ready, False is not ready
    bool contactor_ready; 
    
    //Boolean wether the contactor is connected, True is connected
    bool contactor_status;
  };

  struct SolarInput {
    //The power of the different MPPTs in W
    float MPPT_power[10];
    //The power of the different solar panels in W
    float panel_power[10];
    
  };

  struct DriverInput {
    //Motor and driver temperature in degC
    float motor_temp, driver_temp;
    //Output power in W, motor speed in rads/s
    float driver_output_power, motor_speed;
    //Input voltage of the driver in V, input current in A
    float driver_voltage_input, driver_current_input;
    //The state of the driver, true is on, false is freewheel
    bool driver_state;
    //Test wether the key is in the motor controller, true if motor controller is off
    bool deadman_switch;
  };


  //All the power input data from the batteries
  BatteryInput battery;
  
  //All the input data from the solar panels
  //All the input data from the driver (Motor controller)
  SolarInput solar_panels;

  //All the input data from the driver (Motor controller)
  DriverInput driver;
};

/*
 All the input structure from the user e.g. Steering Wheel and dashboard.
 */
enum FlyMode {
  NO_FLY,
  FLY,
  BRIDGE,
  SLALOM
};

enum PIDState {
  STATE1,
  STATE2,
  STATE3,
  STATE4,
  STATE5,
  STATE6,
  STATE7,
  STATE8
};

struct UserInput {

  /*
   all user input nested into 3 sub parts: 
   * steer (for the steeringwheel)
   * buttons (for the buttons)
   * control (for the Control related inputs (also roll from pedals)
   */

  struct ControlUserInput {
    //Predefined PID states, using an enumerate
    PIDState PID_left;
    PIDState PID_right;
    PIDState PID_back;

    //Roll given by the pedals saved as angle the boat should have
    float roll;
  };

  struct ButtonInput {
    // State from the motor wether the battery should be on
    bool battery_on;
    // Bool stating whether the contactor and balancing of the battery should be forced
    bool force_battery; 
    // bool whether the button for the solar 
    bool solar_on;
    // States whether the system should debug, false on start-up, on true all the software is restarted
    bool debug_on;
  };

  struct SteeringInput {
    //raw value of the Throttle 0 till 1024 (from the analog inputs)
    int raw_throttle;
    //values: NO_FLY, FLY, BRIDGE, SLALOM
    FlyMode fly_mode;
    bool reverse;
  };

  SteeringInput steer;

  ButtonInput buttons;

  ControlUserInput control;
};

/*
 All the data gotten from the Telemetry System
 */
struct TelemetryInput {

  struct TelemetryControlInput {

    struct TelemetryPID {
      // PID structure used in the telemetry system
      float P, I, D, N;
    };

    TelemetryPID PID_height;
    TelemetryPID PID_roll;
    TelemetryPID PID_pitch;
    //Whether the Telemetry system should overwrite the PID values
    bool overwrite;
  };

  TelemetryControlInput control;
  bool solar_panel_states[10];
  float advised_speed;
};

/*
 * structure for all the outputs which are send to Power data
 */
struct PowerOutput {
  // bool storing all the solar panel states
  bool solar_panel_states[10];

  // the throttle used by the Motor from -320000 to 320000
  signed short int throttle;

  // The state of the motor true = on, false = freewheel
  bool motor_state;

  // Control of the contractor:
  // 0 = off
  // 1 = On when BMS want it to be on
  // 2 = Force On
  unsigned char contractor_control;

  // Control of the balancing:
  // 0 = regular balancing
  // 1 = force balancing
  unsigned char balancing_control;

  // Stores if there is any error in the motor:
  // 0 = no error
  // 1 = some error
  unsigned char error;

  //TODO: make error divided into more sub sections.
};

/*
 Structure for the Control system, both all the inputs and all the outputs.
 */
struct ControlData {

  struct InputXSenseData {
    float raw_pitch, raw_roll;
    float filtered_pitch, filtered_roll;
    float raw_z_acceleration;
  };

  struct Vlotters {
    float angle_left, angle_right;
  };

  struct ComputedControlData {
    float force_roll, force_pitch, force_height;
    float angle_left, angle_right, angle_back;
  };

  InputXSenseData xsens;
  Vlotters vlotters;
  ComputedControlData computed;
  float real_height;
  float real_roll;
};
}/* structures */
}/* top_level */
#endif /* STRUCTURES_H */

