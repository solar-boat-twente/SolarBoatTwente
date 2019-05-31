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

const unsigned long CANID_MPPT_START_ADDRESS = 0x53D;
const unsigned long CANID_MPPT_RELAY = 0x53C;

const short int STD_MPPT_DELAY = 50;

const short int NUMBER_OF_MPPTS = 10;

const float A_factor = 44.391;
const float V_factor = 14.2;


}
}
#endif /* MPPT_CANIDS_HPP */

