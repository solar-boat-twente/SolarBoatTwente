/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   BMS.h
 * Author: Sander Oosterveld
 *
 * Created on April 11, 2019, 9:27 PM
 */

#ifndef BMS_H
#define BMS_H

#include <thread>

#include "../../lib-cpp/Canbus/canbus.h"
#include "../../solarboattwente.h"
#include "BMS_CANIDs.h"

namespace MIO{

namespace PowerElectronics{

constexpr short int kMaxCells = 12;
constexpr float kBmsVoltageMultiplier = 0.002; //Precision of data is 2mV
constexpr float kBmsCurrentMultiplier = 0.02; //Precision is 20mA
constexpr float kBmsCellVoltageMultiplier = 0.001; //Precisiono of 1mV



class BMS {
  
 public: 
  struct BmsStatus{
    char can_device[20];

    bool read_state;

    bool write_state;
    };
    
  static const int STD_BMS_READ_DELAY = 500;
  static const int STD_BMS_WRITE_DELAY = 100;
  
  
 public:
  /*
   * Example: 
   *  CANbus * const canbus = new CANbus("can0");
   *  canbus->start();
   *  structures::PowerInput * const power_input = new structures::PowerInput;
   *  BMS * const bms = new BMS(canbus);
   * 
   *  BMS.read_data(power_input);
   * 
   *  BMS.write_data(power_output);
   * 
   *  BMS.stop_reading();
   * 
   *  BMS.stop_writing();
   *  
   */
  BMS(CANbus * const m_canbus, structures::PowerInput * power_input, structures::PowerOutput * power_output);
  
  virtual ~BMS();
  
  /*
   * Public function which starts the reading thread.
   * 
   * Arguments:
   *  pointer to PowerInput structure which it has to read from
   * 
   * Returns:
   *  1 : Start reading is successful
   *  -1: Start reading not successful
   */
  int start_reading(structures::PowerInput * const power_input, short int delay = STD_BMS_READ_DELAY);
  
    
  /*
   * Public function which starts the reading thread.
   * 
   * Arguments:
   *  pointer to PowerInput structure which it has to read from
   * 
   * Returns:
   *  1 : Start writing is successful
   *  -1: Start not successful not successful
   */
  int start_writing(structures::PowerOutput * const power_output, short int delay = STD_BMS_WRITE_DELAY);
  /*
   * Write one message to the can bus (mostly for testing); 
   * 
   * Mimics the canbus function;
   * 
   */
  int write(canmsg_t &msg);
  /*
   * Stops the reading thread
   * 
   * Returns:
   *  1 : success
   *  -1: No Success
   */
  int stop_reading();
  
    /*
   * Stops the writing thread
   * 
   * Returns:
   *  1 : success
   *  -1: No Success
   */
  int stop_writing();
  
  /*
   * Retuns the status of the BMS class
   * 
   * Returns:
   *  Pointer to a BmsStatus structure (see above)
   */
  BmsStatus status();
    
 private:
  int get_data_(structures::PowerInput * const power_input);
  
  int parse_response1_(std::vector<uint8_t> &bytes, structures::PowerInput * const power_input);
  
  int parse_response2_(std::vector<uint8_t> &bytes, structures::PowerInput * const power_input);
  
  int parse_cell_voltages(std::vector<uint8_t> &bytes, structures::PowerInput * const power_input);
   

  int write_data_(structures::PowerOutput * const power_output);
  
  
  
  
  void reading_thread_(structures::PowerInput * const power_input, short int delay = STD_BMS_READ_DELAY);
  
  void writing_thread_(structures::PowerOutput * const power_output, short int delay = STD_BMS_WRITE_DELAY);
    
  CANbus * const canbus_;
  
  BmsStatus bms_status;
  
  std::thread m_reading_thread_;
  
  std::thread m_writing_thread_;

};
}
}
#endif /* BMS_H */
