/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Button_Box.hpp
 * Author: Sander Oosterveld
 *
 * Created on May 14, 2019, 4:27 PM
 */

#ifndef BUTTON_BOX_HPP
#define BUTTON_BOX_HPP

#include "../../lib-cpp/ADAM/ADAM.hpp"
#include "../../solarboattwente.h"
#include <thread>
namespace MIO{
namespace UI{

const short int STD_BUTTON_READ_DELAY = 500;


class ButtonBox{
  
 public:
  ButtonBox(ADAM * const adam);
  
  /**
   * Set the state of the button leds
   * 
   * @param led_number The number of the led you want to turn on value between 0 and 3 where 0 is the motor led.
   * @param state The state the led should change to
   * @return returns 1 if successful 0 otherwise
   */
  int switch_led(int led_number, bool state);
  
  
  /**
   * Get the button state for momentary buttons, starts false, becomes true after one press
   * 
   * @param button_number The number of the button which you want to check, starts at 1 (because motor button is not software) smaller than 3
   * @return button state true is on.
   */
  bool get_button_state(int button_number);
  
  
 private:
  ADAM * const m_adam;
};


class ButtonBoxHandler{
  
 public:
  ButtonBoxHandler(ADAM * const adam, structures::UserInput * const user_input, pthread_mutex_t *m_mutex = NULL);
  
  int start_reading(short int delay = STD_BUTTON_READ_DELAY);
  
  int stop_reading();
  
  static int set_motor_led(bool state);
  
  static int set_battery_led(bool state);
  
  static int set_battery_force_led(bool state);
  
  static int set_solar_led(bool state);
  
 private:
  
  static bool initialized;
  
  bool reading_state;
  
  structures::UserInput * const user_input;
  
  std::thread m_reading_thread;
  
  static ButtonBox * button_box;
  
  int read_buttons_all_();
  
  int reading_thread_(short int delay);
  
};


}
}




#endif /* BUTTON_BOX_HPP */

