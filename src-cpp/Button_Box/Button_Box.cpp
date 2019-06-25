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

ButtonBox::ButtonBox(ADAM * const adam): m_adam(adam) {
  for(int i = 0; i<4; i++){
    initial_button_states[i] = get_button_state(i);
    M_OK<<"SET INITIAL BUTTON STATE "<<i<<" TO: "<<initial_button_states[i];
    switch_led(i, false);
  }
}

int ButtonBox::switch_led(int led_number, bool state) {
  if(led_number<0 or led_number>5){
    M_ERR<<"ENTERED INVALID LED NUMBER"<<led_number;
  } else {
    if(state!=output_button_states[led_number]){
      std::cout<<"WRITTEN A "<<state<<" TO "<<led_number<<std::endl;
      m_adam->write_port(led_number+16, state);
    }
      output_button_states[led_number] = state;

  }

}

bool ButtonBox::get_button_state(int button_number) {
  int result = m_adam->read_counter(button_number*2); //Get the button state from the ADAM
  M_DEBUG<<"RESULT FOR: "<<button_number<<" IS: "<<result;
  
  //Check if the value gotten is logical value
  int previous_value = previous_button_states[button_number];
  if(result-previous_value>=0&&result-previous_value<10){
    previous_button_states[button_number] = result;
    result = result - initial_button_states[button_number]; //Substract original button state from current one
    return (result%2 == 1);
  } else {
    M_WARN<<"INVALID VALUE FOR BUTTON STATE, RETURNING PREVIOUS BUTTON STATE";
    return (previous_value%2==1);
  }
}



ButtonBox * ButtonBoxHandler::button_box = NULL;
bool ButtonBoxHandler::initialized = false;

ButtonBoxHandler::ButtonBoxHandler(ADAM * const adam, structures::UserInput* m_user_input, 
                                   pthread_mutex_t * m_mutex): user_input(m_user_input){
  //Make sure that everybody knows that the ButtonBoxHandler exists;
  
  initialized = true;
  button_box = new ButtonBox(adam);
  m_leds_thread_ = std::thread(&ButtonBoxHandler::leds_thread_, this);
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
    if (state){
      led_states[0] = CONTINUOUS;
    } else {
      led_states[0] = NO_LIGHT;
    }
    return 1;
  } else {
    M_ERR<<"DID NOT WRITE MOTOR LED BECAUSE BUTTON BOX HANDLER NOT INITIALIZED";
    return -1;
  }
}



int ButtonBoxHandler::set_motor_led(ButtonState state){
  if(button_box!=NULL){
    led_states[0] = state;
  }
}


int ButtonBoxHandler::set_solar_led(bool state) {
  if(button_box!=NULL){
    if (state){
      led_states[1] = CONTINUOUS;
    } else {
      led_states[1] = NO_LIGHT;
    }
    return 1;
  } else {
    M_ERR<<"DID NOT WRITE SOLAR LED BECAUSE BUTTON BOX HANDLER NOT INITIALIZED";
  }
}

int ButtonBoxHandler::set_solar_led(ButtonState state) {
  if(button_box!=NULL){
    led_states[1] = state;
  }  
 }


int ButtonBoxHandler::set_battery_led(bool state) {
  if(button_box!=NULL){
    if (state){
      led_states[2] = CONTINUOUS;
    } else {
      led_states[2] = NO_LIGHT;
    }
    return 1;  
  } else {
    M_ERR<<"DID NOT WRITE BATTERY LED BECAUSE BUTTON BOX HANDLER NOT INITIALIZED";
  }
}

int ButtonBoxHandler::set_battery_led(ButtonState state) {
  if(button_box!=NULL){
    led_states[2] = state;
  }
 }


int ButtonBoxHandler::set_battery_force_led(bool state) {
  if(button_box!=NULL){
    if (state){
      led_states[3] = CONTINUOUS;
    } else {
      led_states[3] = NO_LIGHT;
    }
    return 1;  
  } else {
    M_ERR<<"DID NOT WRITE SOLAR FORCE BATTERY BECAUSE BUTTON BOX HANDLER NOT INITIALIZED";
  }
}

int ButtonBoxHandler::set_battery_force_led(ButtonState state){
  if(button_box!=NULL){
    led_states[3] = state;
  }
}

int ButtonBoxHandler::set_mppts(bool state) {
  if(button_box!=NULL){
    if (state){
      led_states[4] = CONTINUOUS;
    } else {
      led_states[4] = NO_LIGHT;
    }
    return 1;  
  } else {
    M_ERR<<"DID NOT WRITE SOLAR FORCE BATTERY BECAUSE BUTTON BOX HANDLER NOT INITIALIZED";
  }
}

int ButtonBoxHandler::set_mppts(ButtonState state){
  if(button_box!=NULL){
    led_states[4] = state;
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

void ButtonBoxHandler::leds_thread_() {
  int counter = 0;
  M_OK<<"STARTING BUTTON BOX WRITING THREAD WITH DELAY: "<<(long)SHORT_BLINK_DELAY;
  while(true){
    counter++;
    for(int i = 0; i<5; i++){
      switch (led_states[i]) {
        case CONTINUOUS:
          button_box->switch_led(i, true);
          break;
        case NO_LIGHT:
          button_box->switch_led(i, false);
          break;
        case BLINK_FAST:
          button_box->switch_led(i, !button_box->output_button_states[i]);
          break;
        case BLINK_SLOW:
          if (counter%SLOW_QUICK_BLINK_RATIO == 1){
            button_box->switch_led(i, !button_box->output_button_states[i]);
          }
          break; 
      } 
    } 
    this_thread::sleep_for(chrono::milliseconds(SHORT_BLINK_DELAY));
  }
}




