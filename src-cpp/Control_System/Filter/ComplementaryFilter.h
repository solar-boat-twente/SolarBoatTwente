/* 
 * File:   ComplementaryFilter.h
 * Author: arjan
 *
 * Created on 1 maart 2019, 11:49
 */

#ifndef COMPLEMENTARYFILTER_H
#define	COMPLEMENTARYFILTER_H

#include "../DataStore.h"
#include "../../Control_System/Vlotter/Vlotter.hpp"

#include <stdio.h>      /* printf */
#include <math.h>       /* sin */



namespace MIO{
namespace control{

const float VLOTTER_LENGTH = 0.7;
const float DISTANCE_BETWEEN_VLOTTER = 1.3;
const float PERIOD = 0.0125;

class ComplementaryFilter {
  
 public:
  /**
   * Object used to calculate the real height combining the data from the vlotters
   * and the XSens
   * 
   * @param filtered_data Pointer to a DataStore object including filtered data
   * @param complementary_data Pointer to DataStore object in which the complemenatary data will be written.
   */
  ComplementaryFilter(DataStore * const control_data, Vlotter * vlotter);
  
  /**
   * Calculate the height from the filtered data and put that in the complemantary DataStore object
   */
  void CalculateRealHeight();

  
 private:
  DataStore * const control_data_;
  Vlotter * const vlotter_;
};
}
}
#endif	/* COMPLEMENTARYFILTER_H */

