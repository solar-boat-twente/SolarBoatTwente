#include <time.h>
#include <algorithm>

#include "solarboattwente.h"

#include "src-cpp/Battery_Magagement_System/BMS.h"
#include "src-cpp/Control_Wheel/Control_Wheel.hpp"
#include "src-cpp/Motor/Motor.hpp"
#include "src-cpp/Button_Box/Button_Box.hpp"
#include "lib-cpp/Logging/thread.h"
#include "src-cpp/Temperature_Sensor/Temperature_Sensor.hpp"

#include "src-cpp/Control_System/DataStore.h"
#include "src-cpp/Control_System/Filtered_data.h"
#include "src-cpp/Control_System/Sensor.h"
#include "src-cpp/Control_System/Filter/ComplementaryFilter.h"
#include "src-cpp/Control_System/PID_caller.h"
#include "src-cpp/Control_System/Force_to_wing_angle.h"
#include "src-cpp/Control_System/EPOS.h"
#include "src-cpp/Control_System/Xsens/Xsens.h"
#include "src-cpp/Control_System/Filter/Filtered_data.h"
#include "src-cpp/Control_System/Calculation/PID_caller.h"
#include "src-cpp/Control_System/Maxon/Force_to_wing_angle.h"

#define MOTOR_MODE 3; //Set the motor mode to rpm controlled
const int SPEED_CORRECTION_FACTOR = 100; //value between 0 and 100, set correct for max current.

using namespace std;
using namespace MIO;
using namespace PowerElectronics;
using namespace UI;
using namespace Control;

//Open up the global structures
structures::PowerInput * power_input = new structures::PowerInput;
structures::PowerOutput * power_output = new structures::PowerOutput;
structures::UserInput * user_input = new structures::UserInput;
structures::ControlData * control_data = new structures::ControlData;
structures::TelemetryInput * telemetry_data = new structures::TelemetryInput;

//Open up the data acquisition parts  
Serial * serial_wheel = new Serial("/dev/steer");
CANbus * canbus_bms = new CANbus("/dev/can1", 1); //CANbus for both the bms and the maxon motors
CANbus * canbus_driver = new CANbus("/dev/can0", 1,STD_BAUD, O_RDWR|O_NONBLOCK); //CANbus for only the motor (has to be nonblocking)
UI::ADAM * adam_6050 = new UI::ADAM("192.168.1.50", 502);
UI::ADAM * adam_6017 = new UI::ADAM("192.168.1.127", 502);


//Initiate the power_output_handler which will ensure that the power_output structure is written
structures::PowerOutputHandler * power_output_handler = new structures::PowerOutputHandler(power_input, power_output, user_input);

//Now start the objects which handle the different components
UI::ButtonBoxHandler * button_box = new UI::ButtonBoxHandler(adam_6050, user_input);
UI::ControlWheel * control_wheel = new UI::ControlWheel(serial_wheel);
PowerElectronics::BMS * bms = new PowerElectronics::BMS(canbus_bms);
PowerElectronics::Motor * motor  = new PowerElectronics::Motor(canbus_driver, RD_SUPPLY1|RD_MOTOR1|RD_MOTOR2|RD_DRIVERSTATE|RD_RANGEREF);

//Create the Threads opbject for the screen
Thread * screen_threads = new Thread;

//Create Temperature Sensor for MIO object
MIO::TemperatureSensor * temp_sens = new MIO::TemperatureSensor;

DataStore * xsens_data = new DataStore();
DataStore * ruwe_data = new DataStore();
DataStore * filtered_data = new DataStore();
DataStore * complementary_data= new DataStore();
DataStore * pid_data = new DataStore();
DataStore * FtoW_data = new DataStore();

RawDataFilter * filter = new RawDataFilter();
ComplementaryFilter * com_filter = new ComplementaryFilter(filtered_data, complementary_data);
PID_caller * PIDAAN = new PID_caller();
ForceToWingAngle * FtoW = new ForceToWingAngle();
Serial * m_serial = new Serial("/dev/xsense", 9600);
xsens::Xsens * m_xsens = new xsens::Xsens();
xsens::Sensor * de_sensor = new xsens::Sensor(xsens_data, ruwe_data);
    
EPOS * maxon1 = new EPOS(canbus_bms,adam_6017,1, FtoW_data);
EPOS * maxon2 = new EPOS(canbus_bms,adam_6017,2, FtoW_data);
EPOS * maxon4 = new EPOS(canbus_bms,adam_6017,4, FtoW_data);



void initiate_structures(){
  //Define required parts of the structures:
  user_input->buttons.battery_on = false;
  user_input->buttons.force_battery = false;
  user_input->buttons.solar_on = false;
  user_input->control.PID_roll = structures::STATE1;
  
  control_data->real_height = 0;
  control_data->real_roll = 0;
  control_data->xsens.speed = 0;
  
  
  power_input->driver.motor_temp = 20;
  power_input->driver.driver_temp = 20;
  power_input->battery.max_temp = 20;
  
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
  return true;
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

void generate_driver_message(canmsg_t * driver_tx, int speed){
  driver_tx->id = 0xcf;
  driver_tx->length = 4;
  driver_tx->data[0] = 0;
  driver_tx->data[1] = MOTOR_MODE;
  driver_tx->data[2] = speed>>8;
  driver_tx->data[3] = speed&0xFF;
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
  control_data->real_roll=power_input->battery.total_current*power_input->battery.total_voltage*-1/10;
  control_data->real_height=power_input->battery.state_of_charge;
  if (control_data->real_roll<0){
    control_data->real_roll = -control_data->real_roll;  
  }
  control_data->xsens.speed = (float)get_speed()/32768 * 100;
  if(control_data->xsens.speed<0){
    control_data->xsens.speed = -control_data->xsens.speed;
  }
  if(motor->values.driver_temp!=255){
    power_input->driver.driver_temp = motor->values.driver_temp;
  } else {
    power_input->driver.driver_temp = 1;
  }
  
  power_input->driver.motor_temp = max<float>(temp_sens->get_core_temp_1(),temp_sens->get_core_temp_2());
  power_input->battery.max_temp = temp_sens->get_temp_zone();
  screen_threads->writeUserPower(power_input, power_output, user_input, 0, 1);
  screen_threads->writeControlData(control_data,0,1);
}

void sensor(){
  m_xsens->m_xsens_state_data = xsens_data; //(0)
  uint8_t msg[50];
  uint8_t msg_[350];
  while (true){
    m_serial->read_bytes(msg_, 350);
    //serie->read_bytes(msg_bytes, 35);
    //serie->read_bytes(msg_bytes, 35);
    m_xsens->parse_message(msg_);
    m_xsens->parse_data();  

  }    
}

void controlsystem(){ 
  typedef std::chrono::microseconds ms;
  typedef std::chrono::milliseconds mls;
    /* -----------------------------------------------------------------------------
  All three motors are going to home.
  ----------------------------------------------------------------------------- */    
  this_thread::sleep_for(chrono::milliseconds(2000));
  maxon1->start_homing();
  this_thread::sleep_for(chrono::milliseconds(2000));
  maxon2->start_homing();
  this_thread::sleep_for(chrono::milliseconds(2000));
  maxon4->start_homing();
  this_thread::sleep_for(chrono::milliseconds(500));

  maxon1->check_homing();
  maxon2->check_homing();
  maxon4->check_homing();

  /* -----------------------------------------------------------------------------
  All three motors are going in the startpositionmode.
  ----------------------------------------------------------------------------- */    
  maxon1->start_position_mode();
  this_thread::sleep_for(chrono::milliseconds(1000));      
  maxon2->start_position_mode();
  this_thread::sleep_for(chrono::milliseconds(1000));
  maxon4->start_position_mode();
  this_thread::sleep_for(chrono::milliseconds(1000));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // wait 500 milliseconds, because otherwise the xsens can enter the configuration mode

  filter->add_data(ruwe_data, filtered_data);
  
  PIDAAN->add_data(complementary_data, pid_data);
  
  FtoW->add_data(pid_data, FtoW_data, complementary_data, xsens_data);


  //Serial * m_serial = new Serial("/dev/xsense", 9600);
  //Xsens * m_xsens = new Xsens(); //initialise class Xsens
  //Sensor * de_sensor = new Sensor(m_xsens,m_serial);

  //m_xsens->m_xsens_state_data = xsens_data; //(0)

  //de_sensor->start_receiving();
  int counter_control_front = 4;        //80Hz waar de loop op loopt
  //int counter_control_back = 8;
  while (true) {
    std::chrono::high_resolution_clock::time_point t1= std::chrono::high_resolution_clock::now();
    de_sensor->get_data();
    
    if (counter_control_front == 0){     //Voorvleugels lopen op 20Hz
      filter->filter_data();
      com_filter->CalculateRealHeight();            
      PIDAAN->PID_in();
      FtoW->MMA();
      
      maxon1->move();
      maxon2->move(); 
      maxon4->move();
      this_thread::sleep_for(chrono::milliseconds(5));
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
  time_t start = time(0);
  
  std::thread thread_controlsystem(controlsystem);
  std::thread thread_sensor(sensor);
  
  //Leds us first make the led blink before we do anything else
  blink_leds();
  
  //Then we can initiate all the structures
  initiate_structures();
  initiate_components();
  initiate_screen();
  
  //Initiate the driver_tx message
  canmsg_t * driver_tx = new canmsg_t;
  generate_driver_message(driver_tx, 0);
   
  //Initiate the two files for logging
  ofstream file;
  file.open("/root/logfiles/power_log_test_06_07.csv", std::ios::app);
  
  ofstream driver_file;
  driver_file.open("/root/logfiles/driver_log_test_06_07.csv", std::ios::app);
  
  ofstream control_file;
  control_file.open("/root/logfiles/control_log_test_06_07", std::ios::app);
  
  int loop_counter = 0;
  //buffers for driver data;
  bool motor_state = false;
  
  //Wait for a button to be pressed
  while(!(user_input->buttons.battery_on||user_input->buttons.solar_on)){
    this_thread::sleep_for(chrono::milliseconds(100));
  }
  stop_leds();
  while (true){
    loop_counter++;
    this_thread::sleep_for(chrono::milliseconds(50));

    //Get current speed and update driver_tx
    generate_driver_message(driver_tx, get_speed());    
    
    if (motor_state){
        canbus_driver->write_can(driver_tx);
    }
    
    update_leds();
    
    if (loop_counter%10 == 1){
            
      motor_state = update_motor();    
      
      driver_file<<motor_state<<","<<seconds_since_start(start)<<","<<(int)motor->values.driver_temp<<","<<motor->values.link_voltage<<","<<motor->values.phase_current<<","<<motor->values.motor_power<<","<<motor->values.rotor_speed
          <<","<<motor->values.supply_current<<","<<motor->values.supply_voltage<<","<<motor->values.torque<<","<<(int)motor->values.motor_mode<<","<<(int)get_speed()<<"\n"<<flush; 
      
      file<<seconds_since_start(start)<<","<<power_input->battery.max_temp << ","<<power_input->battery.total_current << ","<<power_input->battery.total_voltage << ","
          <<power_input->battery.state_of_charge << ","<<user_input->steer.raw_throttle<<"\n"<<flush;
      
     DataStore *m_PID_data;
     DataStore *m_complementary_data;
     DataStore *m_xsens_state_data;
     
     DataStore::PIDDataTotal log_OUTPUT_PID  = m_PID_data-> GetPIDData(); 
     DataStore::RealData log_INPUT_PID = m_complementary_data-> GetComplementaryData();
     DataStore::XsensData log_xsens = m_xsens_state_data->GetXsensData();
     
     control_file<<seconds_since_start(start)<<log_OUTPUT_PID.Force_height << ","<<log_OUTPUT_PID.Force_pitch << ","<<log_OUTPUT_PID.Force_roll << ","
         <<log_INPUT_PID.Real_height << ","<<log_INPUT_PID.Real_pitch<< ","<<log_INPUT_PID.Real_roll<< ","
         <<log_xsens.roll<< ","<<log_xsens.acceleration_z<< ","<<log_xsens.acceleration_x<<"\n"<<flush;
     
  
  
      
      
      write_to_screen();
    } 
  }
  return 0;
}