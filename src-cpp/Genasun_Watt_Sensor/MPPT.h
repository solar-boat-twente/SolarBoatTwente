/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MPPT.h
 * Author: Sander Oosterveld
 *
 * Created on April 18, 2019, 1:49 PM
 */

#ifndef MPPT_H
#define MPPT_H


#include <thread>
#include <stdlib.h>
#include <stdint.h>

#include "../../lib-cpp/Canbus/canbus.h"
#include "MPPT_CANIDs.hpp"
#include "../../solarboattwente.h"

namespace MIO{
namespace PowerElectronics{

/*
 * Class to handle the data for the MPPT sensors, everything here was self written
 * 
 * @example Example use (gets the Bat voltage of the 3rd MPPT):
 *  CANbus * const canbus = new CANbus("can0");
 *  MPPT * const mppt = new MPPT(canbus);
 *  
 *  //buffer must be at least 4 long because one adress results in 4 floats of data
 *  float data[8]
 *  mppt->get_float_from_address(data, 0x3F);
 *  
 *  //The data order is PV_Volt, PV_AMP, Bat_Volt, Bat_AMP so we need the 3rd value
 *  float bat_voltage_3 = data[2];
 * 
 * Example use (Get all the PV amperages for 5 cells start address is 0x3D):
 *  CANbus * const canbus = new CANbus("can0");
 *  MPPT * const mppt = new MPPT(canbus, 5);
 *  
 *  //The data will be stored in a matrix where the first number is the number of the cell; 
 *  float buffer[5][4]
 *  mppt->get_all_float_data(buffer);
 * 
 *  //The PV amp is the 2nd value so we can get them like this:
 *  float PV_amps[5];
 *  for(int i = 0; i<5; i++){
 *    PV_amps[i] = buffer[i][1]
 *  }
 * 
 * Example use (turn of the relay for mppt 5 and 8, int start_address = 0x3D);
 * 
 *  CANbus * const canbus = new CANbus("can0");
 *  MPPT * const mppt = new MPPT(canbus);
 *  
 *  mppt->set_relay_from_number(false, 5);
 *  mppt->set_relay_from_number(false, 8);
 *  
 */
class MPPT_Box{
  
 public:
  
  friend class MPPT_Controller;
  
  /**
   * Construct the MPPT object, you have to supply a canbus, all other parameters are already preset
   * @param canbus A pointer to a CANbus object, make sure that this canbus is opened/started before you try anything
   * @param number_of_mppts The number of MPPTs connected to the CANbus
   * @param start_address The Start adress where the MPPTs start writing
   * @param relay_address The address with which you can write the relays.
   */
  MPPT_Box(CANbus * const canbus, uint8_t number_of_mppts = 10, unsigned long start_address = CANID_MPPT_START_ADDRESS, unsigned long relay_address = CANID_MPPT_RELAY);
  
  
  /**
   * Receives the raw bytes from the MPPT for a certain CANbus address. These
   * Are written to the supplied buffer
   * 
   * @param buf Buffer of uint_8  in which the data is written, should be at least 8 long
   * @param address The address from which you would like to see the message ex: 0x3E
   * @return 1 for success, 0 if the bytes has been read before, -1 when failure
   */
  int get_bytes_from_address(uint8_t buf[], unsigned long address);
  
  /**
   * Receives the raw bytes for a certain MPPT unit, for this it uses the
   * start_address given in the constructor
   * 
   * @param buf Buffer of uint_8 in which the data is written, should be at least 8 long
   * @param cell_number The number for which you want to know the data (1 till number of mppts)
   * @return 1 for success, 0 if the bytes has been read before, -1 when failure
   */
  int get_bytes_from_number(uint8_t buf[], int cell_number);
  
  /**
   * Receives the float value for the voltage and current and stores it in the supplied
   * buffer, requires the address of the cell you want to know
   * 
   * @param buf Buffer of float in which the data is written, should be at least 4 long
   * @param address The address from which you would like to see the message ex: 0x3E
   * @return 1 for success, 0 if the bytes has been read before, -1 when failure
   */
  int get_float_from_address(float buf[], unsigned long address);
  
  /**
   * Receives the float value for the voltage and current and stores it in the supplied
   * buffer, requires the number of the cell you want to know the data from
   * 
   * @param buf Buffer of float in which the data is written, should be at least 4 long
   * @param cell_number The address from which you would like to see the message ex: 0x3E
   * @return 1 for success, 0 if the bytes has been read before, -1 when failure
   */  
  int get_float_from_number(float buf[], int cell_number);
  
  
  /**
   * Receives the data from all of the bytes, uses the data from the constructor
   * to find the respective addresses. Writes this data in the buffer in a uint_8
   * 
   * @param buf Buffer of u_int8 in which the data is written, this should be an matrix with the size:
   * [number_of_cells][8]
   * @return number of cells read, if some of them have been
   * read before this number is smaller than number_of_cells defined in the constructor
   * Returns -1 in case of failure
   * 
   */
  int get_all_bytes_data(uint8_t buf[][8]);
  
  
  /**
   * Receives the data from all of the bytes, uses the data from the constructor
   * to find the respective addresses, writes this data in the buffer as floats
   * 
   * @param buf Buffer of floats in which the data is written, this should be an matrix with the size:
   * [number_of_cells][4]
   * @return umber of cells read, if some of them have been
   * read before this number is smaller than number_of_cells defined in the constructor
   * Returns -1 in case of failure
   */
  int get_all_float_data(float buf[][4]);
  
  /**
   * Sets the relay for a certain address -> for this the start address is used to get the number of the relay
   * If you know the number of the relay the set_relay_from_number is therefore faster
   * If you want to switch multiple relays please use an array of adresses
   * 
   * @param state State you want to set the relays in;
   * @param address Address of the Relay you want to change;
   * @return Returns 1 when success, 0 otherwise
   */
  int set_relay_from_address(bool state, unsigned long address);
  int set_relay_from_address(bool state, unsigned long address[], int number_of_cells);

  /**
   * Sets the relay for a number. If you want to switch multiple relays please
   * use an array of adresses
   * 
   * @param state State you want to set the relays in;
   * @param cell_number Address of the Relay you want to change;
   * @return Returns 1 when success, 0 otherwise
   */
  int set_relay_from_number(bool state, int cell_number); 
  int set_relay_from_number(bool state, int cell_number[], int number_of_cells);

  /**
   * Switches all the relays into a certain state, 'all' is defined in the 
   * constructor 
   * 
   * @param state State you want to set the relays in;
   * @return Returns number of relays switched, 
   */
  int set_all_relay(bool state);
  
  /**
   * 
   * @param state A array of booleans with different states for the different MPPTs
   * the length of state should be less than number of mppts.
   * @return 
   */
  int set_relay_from_array(bool state[]);
  
  int set_relay_states_from_number_(int cell_number, bool state);  
 private: 
  
  int set_relay_from_relay_states_();
  
  int get_int_(uint8_t high_byte, uint8_t low_byte);
  
  float get_A_float(uint8_t high_byte, uint8_t low_byte);
  
  float get_V_float(uint8_t high_byte, uint8_t low_byte);
  


  
  unsigned long start_address_;
  
  unsigned long relay_address_;
  
  const int number_of_mppts_;
  
  CANbus *  canbus_ = NULL;
  
  uint8_t relay_states[2] = {0, 0};
  
  
};


class MPPT_Controller{
  
  
 public:
  MPPT_Controller(MPPT_Box * const m_mppt, structures::PowerInput* const m_power_input, structures::PowerOutput* const m_power_output);
  
  int start_reading(const int delay = STD_MPPT_DELAY);
  
  int stop_reading();
  
  int start_writing(const int delay = STD_MPPT_DELAY);
  
  int stop_writing();
  
 private:
  
  int read_data_();
  
  int write_data_();
  
  int reading_thread_(const int delay);
  
  int writing_thread_(const int delay);
  
  MPPT_Box * const mppt;
  
  structures::PowerInput * const power_input;
  structures::PowerOutput * const power_output;
  
  bool reading_state;
  bool writing_state;
  
  std::thread m_reading_thread_;
  std::thread m_writing_thread_;
  
  
  
};

}/*PowerElectronics*/
} /*MIO*/


#endif /* MPPT_H */
