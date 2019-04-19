/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MPPT_CANIDs.hpp
 * Author: Sander Oosterveld
 *
 * Created on April 18, 2019, 3:32 PM
 */

#ifndef MPPT_CANIDS_HPP
#define MPPT_CANIDS_HPP

namespace MIO{
namespace PowerElectronics{

const unsigned long CANID_MPPT_START_ADDRESS = 0x3D;
const unsigned long CANID_MPPT_RELAY = 0x3F;
  
}
}
#endif /* MPPT_CANIDS_HPP */

