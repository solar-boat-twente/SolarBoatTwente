/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "Button_Box.hpp"
#include "../../lib-cpp/Debugging/easy_debugging.hpp"
#include <thread>

using namespace std;

using namespace MIO::UI;

/** TO DO: MAKE SURE THAT THIS STARTS AT FALSE ON START UP (STORE FIRST VALUE)*/
ButtonBox::ButtonBox(ADAM * const adam): m_adam(adam) {
    }

int ButtonBox::switch_led(int led_number, bool state) {
  if(led_number<0 or led_number>4){
    M_ERR<<"ENTERED INVALID LED NUMBER"<<led_number;
  }
  std::cout<<"WRITTEN A "<<state<<" TO "<<led_number<<std::endl;
  m_adam->write_port(led_number+16, state);

}

bool ButtonBox::get_button_state(int button_number) {
  int result = m_adam->read_counter(button_number*2);
  M_DEBUG<<"RESULT FOR: "<<button_number<<" IS: "<<result;
  return (m_adam->read_counter(button_number*2)%2 == 1);
}

ButtonBox * ButtonBoxHandler::button_box = NULL;
bool ButtonBoxHandler::initialized = false;

ButtonBoxHandler::ButtonBoxHandler(ADAM * const adam, structures::UserInput* m_user_input, 
                                   pthread_mutex_t * m_mutex): user_input(m_user_input){
  //Make sure that everybody knows that the ButtonBoxHandler exists;
  
  initialized = true;
  button_box = new ButtonBox(adam);
}

int ButtonBoxHandler::start_reading(short int delay) {
  
  m_reading_thread = std::thread(&ButtonBoxHandler::reading_thread_, this, delay);
  M_OK<<"STARTED READING THREAD WITH DELAY: "<< delay;
}

int ButtonBoxHandler::stop_reading() {
  reading_state = false;
  m_reading_thread.join();  
}



int ButtonBoxHandler::set_motor_led(bool state) {
  if(button_box!=NULL){
    button_box->switch_led(0, state);
  } else {
    M_ERR<<"DID NOT WRITE MOTOR LED BECAUSE BUTTON BOX HANDLER NOT INITIALIZED";
  }
}

int ButtonBoxHandler::set_solar_led(bool state) {
  if(button_box!=NULL){
    button_box->switch_led(1, state);
  } else {
    M_ERR<<"DID NOT WRITE SOLAR LED BECAUSE BUTTON BOX HANDLER NOT INITIALIZED";
  }
}

int ButtonBoxHandler::set_battery_led(bool state) {
  if(button_box!=NULL){
    button_box->switch_led(2, state);
  } else {
    M_ERR<<"DID NOT WRITE BATTERY LED BECAUSE BUTTON BOX HANDLER NOT INITIALIZED";
  }
}

int ButtonBoxHandler::set_battery_force_led(bool state) {
  if(button_box!=NULL){
    button_box->switch_led(3, state);
  } else {
    M_ERR<<"DID NOT WRITE SOLAR FORCE BATTERY BECAUSE BUTTON BOX HANDLER NOT INITIALIZED";
  }
}

int ButtonBoxHandler::read_buttons_all_() {
  
  user_input->buttons.solar_on = button_box->get_button_state(1);
  user_input->buttons.battery_on = button_box->get_button_state(2);
  user_input->buttons.force_battery = button_box->get_button_state(3);
  
}

int ButtonBoxHandler::reading_thread_(short int delay) {
  M_OK<<"STARTING BUTTON BOX READING THREAD WITH DELAY: "<<(long)delay;
  reading_state = true;
  while(reading_state){
    read_buttons_all_();
    this_thread::sleep_for(chrono::milliseconds(delay));    
  }
}





