#include <time.h>
#include <algorithm>
#include <sys/ioctl.h>

#include "solarboattwente.h"

#include "src-cpp/Battery_Magagement_System/BMS.h"
#include "src-cpp/Control_Wheel/Control_Wheel.hpp"
#include "src-cpp/Motor/Motor.hpp"
#include "src-cpp/Button_Box/Button_Box.hpp"
#include "lib-cpp/Logging/thread.h"
#include "src-cpp/Temperature_Sensor/Temperature_Sensor.hpp"

#include "src-cpp/Control_System/DataStore.h"
#include "src-cpp/Control_System/Filter/Filtered_data.h"
#include "src-cpp/Control_System/Xsens/Sensor.h"
#include "src-cpp/Control_System/Filter/ComplementaryFilter.h"
#include "src-cpp/Control_System/Calculation/PID_caller.h"
#include "src-cpp/Control_System/Maxon/EPOS.h"
#include "src-cpp/Control_System/Xsens/Xsens.h"
#include "src-cpp/Control_System/Filter/Filtered_data.h"
#include "src-cpp/Control_System/Calculation/PID_caller.h"
#include "src-cpp/Control_System/Maxon/Force_to_wing_angle.h"
#include "src-cpp/Control_System/Vlotter/Vlotter.hpp"

#define MOTOR_MODE 3; //Set the motor mode to rpm controlled
const int SPEED_CORRECTION_FACTOR = 100; //value between 0 and 100, set correct for max current.

using namespace std;
using namespace MIO;
using namespace PowerElectronics;
using namespace UI;
using namespace control;

//Open up the global structures
structures::PowerInput * power_input = new structures::PowerInput;
structures::PowerOutput * power_output = new structures::PowerOutput;
structures::UserInput * user_input = new structures::UserInput;
structures::ControlData * logging_control_data = new structures::ControlData;
structures::TelemetryInput * telemetry_data = new structures::TelemetryInput;

//Open up the data acquisition parts 
//Serial * vlotter_serial = new Serial("/dev/ttyACM0");
//    Control::Vlotter vlotter(m_serial);
//    vlotter.start_reading(10);
Serial * serial_wheel = new Serial("/dev/steer");
CANbus * canbus_bms = new CANbus("/dev/can1", 1); //CANbus for both the bms and the maxon motors
CANbus * canbus_driver = new CANbus("/dev/can0", 1, STD_BAUD, O_RDWR|O_NONBLOCK); //CANbus for only the motor (has to be nonblocking)
//CANbus * canbus_driver = new CANbus("/dev/can0", 1); //CANbus for only the motor (has to be nonblocking)

UI::ADAM * adam_6050 = new UI::ADAM("192.168.1.50", 502);
UI::ADAM * adam_6017 = new UI::ADAM("192.168.1.127", 502);


//Initiate the power_output_handler which will ensure that the power_output structure is written
structures::PowerOutputHandler * power_output_handler = new structures::PowerOutputHandler(power_input, power_output, user_input);

//Now start the objects which handle the different components
UI::ButtonBoxHandler * button_box = new UI::ButtonBoxHandler(adam_6050, user_input);
UI::ControlWheel * control_wheel = new UI::ControlWheel(serial_wheel);
PowerElectronics::BMS * bms = new PowerElectronics::BMS(canbus_bms, power_input, power_output);
PowerElectronics::Motor * motor  = new PowerElectronics::Motor(canbus_driver, RD_SUPPLY1|RD_MOTOR1|RD_MOTOR2|RD_DRIVERSTATE|RD_RANGEREF);

//Create the Threads opbject for the screen
Thread * screen_threads = new Thread;

//Create Temperature Sensor for MIO object
MIO::TemperatureSensor * temp_sens = new MIO::TemperatureSensor;
Serial * serial_vlotter = new Serial("/dev/ttyACM0");
Vlotter * vlotter = new Vlotter(serial_vlotter);

/*
DataStore * xsens_data = new DataStore();
DataStore * ruwe_data = new DataStore();
DataStore * filtered_data = new DataStore();
DataStore * complementary_data= new DataStore();
DataStore * pid_data = new DataStore();
DataStore * FtoW_data = new DataStore();
*/

DataStore * control_data = new DataStore();

RawDataFilter * raw_data_filter = new RawDataFilter(control_data);
ComplementaryFilter * complementary_filter = new ComplementaryFilter(control_data, vlotter);
PID_caller * pid_caller = new PID_caller(control_data);
ForceToWingAngle * force_to_wing = new ForceToWingAngle(control_data);

Serial * serial_xsens = new Serial("/dev/xsense", 9600);
xsens::Xsens * xsens_object = new xsens::Xsens(control_data);

xsens::Sensor * sensor = new xsens::Sensor(control_data, vlotter);

time_t now = time(nullptr);

EPOS * maxon_right = new EPOS(canbus_bms, adam_6017, 1, control_data, now);
EPOS * maxon_left = new EPOS(canbus_bms,adam_6017, 2, control_data, now);
EPOS * maxon_back = new EPOS(canbus_bms,adam_6017,4, control_data, now);


void initiate_structures(){
  //Define required parts of the structures:
  user_input->buttons.battery_on = false;
  user_input->buttons.force_battery = false;
  user_input->buttons.solar_on = false;
  user_input->control.PID_roll = structures::STATE1;
  
  logging_control_data->real_height = 0;
  logging_control_data->real_roll = 0;
  logging_control_data->xsens.speed = 0;
  
  power_input->battery.contactor_status = false;
  
  power_input->driver.motor_temp = 20;
  power_input->driver.driver_temp = 20;
  power_input->battery.max_temp = 20;
  power_input->driver.motor_speed = 0;
  
  telemetry_data->advised_speed = 0;
}

void blink_leds(){
  button_box->set_battery_force_led(UI::BLINK_SLOW);
  button_box->set_battery_led(UI::BLINK_SLOW);
  button_box->set_motor_led(UI::BLINK_SLOW);
  button_box->set_solar_led(UI::BLINK_SLOW);
}

void stop_leds(){
  button_box->set_battery_force_led(UI::NO_LIGHT);
  button_box->set_battery_led(UI::NO_LIGHT);
  button_box->set_motor_led(UI::NO_LIGHT);
  button_box->set_solar_led(UI::NO_LIGHT);
}

void update_leds(){
  //Static led states to check if we need to even update the led state or not
  static UI::ButtonState previous_solar_led_state = BLINK_SLOW;
  static UI::ButtonState previous_battery_led_state = BLINK_SLOW;
  static UI::ButtonState previous_force_led_state = BLINK_SLOW;
  
  UI::ButtonState solar_led_state;
  UI::ButtonState battery_led_state;
  UI::ButtonState force_led_state;
  
  if(user_input->buttons.solar_on){
    //canbus_driver->write_can(mppt_on);
    solar_led_state = CONTINUOUS;
  } else {
    //canbus_driver->write_can(mppt_off);
    solar_led_state = NO_LIGHT;
  }
   
  if(user_input->buttons.force_battery){
    force_led_state = CONTINUOUS;
  } else {
    force_led_state = NO_LIGHT;
  }
  
  if(user_input->buttons.battery_on){
    battery_led_state = BLINK_FAST;
  } else {
    battery_led_state = NO_LIGHT;
  }
  
  if(power_input->battery.contactor_status){
    battery_led_state = CONTINUOUS;
  }
  
  
  //Update the led states
  if(solar_led_state!=previous_solar_led_state){
    button_box->set_solar_led(solar_led_state);
    previous_solar_led_state = solar_led_state;
  }
  
  if(battery_led_state!=previous_battery_led_state) {
    button_box->set_battery_led(battery_led_state);
    previous_battery_led_state= battery_led_state;
  }
  
  if(force_led_state!=previous_force_led_state) {
    button_box->set_battery_force_led(force_led_state);
    previous_force_led_state = force_led_state;
  }
}

bool update_motor(){
  static int no_driver_data_counter = 5;
  if(motor->update_data_from_driver()){
    no_driver_data_counter = 0;
    button_box->set_motor_led(true);
    return true;
  } else {
    no_driver_data_counter++;
  }
  if(no_driver_data_counter>5){
    motor->setup_sampling();
    no_driver_data_counter = 4;
    button_box->set_motor_led(false);
    return false;

  }
  return false;
}

void initiate_components(){
  //Starting with reading from the CANbus
  canbus_bms->start(0);
  canbus_driver->start(0);
  
   
  //Start writing the power_output structures with delay 50
  power_output_handler->start(50);
  
  //Start the Steering Wheel with delay 50
  control_wheel->start_reading(user_input, 50);
  
  //Start reading BMS with a speed just a bit faster than data is send 1.4
  //and start writing with delay of 400 ms (which is faster than required but who cares)
  bms->start_reading(power_input, 1300);
  bms->start_writing(power_output, 400);
  
  //Start reading the button box every 300 ms
  button_box->start_reading(300);
}

void initiate_screen(){
  screen_threads->CreateThreads();
}

void generate_driver_message(canmsg_t & driver_tx, int speed){
  driver_tx.id = 0xcf;
  driver_tx.length = 4;
  driver_tx.data[0] = 0;
  driver_tx.data[1] = MOTOR_MODE;
  driver_tx.data[2] = speed>>8;
  driver_tx.data[3] = speed&0xFF;
}

int get_speed(){
  static int real_speed;
  real_speed = 32 * user_input->steer.raw_throttle * SPEED_CORRECTION_FACTOR/100;
  if (user_input->steer.reverse){
    real_speed = -real_speed;
  }
  return real_speed;
}

double seconds_since_start(time_t start){
  return difftime(time(0), start);
}

void write_to_screen(){
  //Write power data to the control structure for reading it on the screen.
  logging_control_data->real_roll=power_input->battery.total_current*power_input->battery.total_voltage*-1/10;
  logging_control_data->real_height=power_input->battery.state_of_charge;
  if (logging_control_data->real_roll<0){
    logging_control_data->real_roll = -logging_control_data->real_roll;  
  }
  logging_control_data->xsens.speed = (float)get_speed()/32768 * 100;
  if(logging_control_data->xsens.speed<0){
    logging_control_data->xsens.speed = -logging_control_data->xsens.speed;
  }
  if(motor->values.driver_temp!=255){
    power_input->driver.driver_temp = motor->values.driver_temp;
  } else {
    power_input->driver.driver_temp = 1;
  }
  
  power_input->driver.motor_temp = max<float>(temp_sens->get_core_temp_1(),temp_sens->get_core_temp_2());
  power_input->battery.max_temp = temp_sens->get_temp_zone();
  screen_threads->writeUserPower(power_input, power_output, user_input, 0, 1);
  screen_threads->writeControlData(logging_control_data,0,1);
}

void sensor_thread(){
  uint8_t msg_[350];
  while (true){
    serial_xsens->read_bytes(msg_, 350);

    if(xsens_object->parse_message(msg_)){ 
      xsens_object->parse_data();
    }
    this_thread::sleep_for(chrono::milliseconds(10));

  }    
}

void controlsystem(){ 
  typedef std::chrono::microseconds ms;
  typedef std::chrono::milliseconds mls;
    /* -----------------------------------------------------------------------------
  All three motors are going to home.
  ----------------------------------------------------------------------------- */    
  maxon_right->start_homing(true);
  maxon_left->start_homing(true);
  maxon_back->start_homing(false);

  maxon_right->check_homing();
  maxon_left->check_homing();
  maxon_back->check_homing();

  /* -----------------------------------------------------------------------------
  All three motors are going in the startpositionmode.
  ----------------------------------------------------------------------------- */    
  maxon_right->start_position_mode();
  maxon_left->start_position_mode();
  maxon_back->start_position_mode();


  int counter_control_front = 4;        //80Hz waar de loop op loopt
  while (true) {
    std::chrono::high_resolution_clock::time_point t1= std::chrono::high_resolution_clock::now();
    sensor->update_data();
    
    if (counter_control_front == 0){     //Voorvleugels lopen op 20Hz
      raw_data_filter->filter_data();
      complementary_filter->CalculateRealHeight();            
      pid_caller->PID_in();
      force_to_wing->MMA(power_input);
      
      maxon_right->move();
      maxon_left->move(); 
      maxon_back->move();
      counter_control_front = 5;
    }
//    if (counter_control_back == 0){ //achtervleugel op 10 Hz
//      filter->FilterIt();
//      com_filter->CalculateRealHeight();            
//      PIDAAN->PID_in();
//      FtoW->MMA();  
//      maxon4->Move();
//      counter_control_back=9;
//    }
    counter_control_front--;
//    counter_control_back--;
    std::chrono::high_resolution_clock::time_point t2= std::chrono::high_resolution_clock::now();
    ms d = std::chrono::duration_cast<ms>(t2-t1);
    int t_total = 12500-d.count();
    printf("loop tijd is %i \r\n",t_total);
    this_thread::sleep_for(chrono::microseconds(t_total));
  }
}

int main(int argc, const char** argv) { 
  typedef std::chrono::microseconds ms;
  typedef std::chrono::milliseconds mls;
  time_t start = time(nullptr);
  
  std::thread thread_controlsystem(controlsystem);
  canbus_bms->start(0);
  canbus_driver->start(0);
  std::thread thread_sensor(sensor_thread);
  
  //Leds us first make the led blink before we do anything else
  blink_leds();
  
  //Then we can initiate all the structures
  initiate_structures();
  initiate_components();
  initiate_screen();
  
  //Initiate the driver_tx message
  canmsg_t driver_tx;
  generate_driver_message(driver_tx, 0);
   
  //Initiate the two files for logging
  ofstream file;
  file.open("/root/logfiles/power_log_test_06_14.csv", std::ios::app);
  
  ofstream driver_file;
  driver_file.open("/root/logfiles/driver_log_test_06_14.csv", std::ios::app);
  
  ofstream control_file;
  control_file.open("/root/logfiles/control_log_test_06_13.csv", std::ios::app);
  
  int loop_counter = 0;
  //buffers for driver data;
  bool motor_state = false;
  
  //Wait for a button to be pressed
  while(!(user_input->buttons.battery_on||user_input->buttons.solar_on)){
    this_thread::sleep_for(chrono::milliseconds(100));
  }
  stop_leds();
  std::chrono::high_resolution_clock::time_point t1, t2;
  ms duration;
  while (true){
    
    t1 = std::chrono::high_resolution_clock::now();

    loop_counter++;
    this_thread::sleep_for(chrono::milliseconds(30));

    //Get current speed and update driver_tx
    generate_driver_message(driver_tx, get_speed());    
    
    if (motor_state){
        canbus_driver->write_can(driver_tx);
    }
    
    update_leds();
    
    if(loop_counter%30 == 1){
      motor_state = update_motor();
      if (motor_state==false){
        ioctl(canbus_driver->get_status().file_descriptor, TCFLSH, 1);
      }
    }
    
    if (loop_counter%10 == 1){
      //adam_6017->read_counter(5);
      //motor_state = update_motor();  
 
      
      power_input->driver.motor_speed = (float)motor->values.rotor_speed;
      driver_file<<motor_state<<","<<seconds_since_start(start)<<","<<(int)motor->values.driver_temp<<","<<motor->values.link_voltage<<","<<motor->values.phase_current<<","<<motor->values.motor_power<<","<<motor->values.rotor_speed
          <<","<<power_input->driver.motor_speed/216<<","<<motor->values.supply_current<<","<<motor->values.supply_voltage<<","<<motor->values.torque<<","<<(int)motor->values.motor_mode<<","<<(int)get_speed()<<"\n"<<flush; 
      
      file<<seconds_since_start(start)<<","<<power_input->battery.max_temp << ","<<power_input->battery.total_current << ","<<power_input->battery.total_voltage << ","
          <<power_input->battery.state_of_charge << ","<<user_input->steer.raw_throttle<<"\n"<<flush;

      write_to_screen();
    } 
   }
  return 0;
}