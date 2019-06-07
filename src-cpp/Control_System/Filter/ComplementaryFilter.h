/* 
 * File:   ComplementaryFilter.h
 * Author: arjan
 *
 * Created on 1 maart 2019, 11:49
 */

#ifndef COMPLEMENTARYFILTER_H
#define	COMPLEMENTARYFILTER_H

#include "../DataStore.h"

#include <stdio.h>      /* printf */
#include <math.h>       /* sin */
namespace MIO{
namespace Control{

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
  ComplementaryFilter(DataStore * filtered_data, DataStore * complementary_data)
    : m_filtered_data(filtered_data), m_complementary_data(complementary_data) {
      
    };
  ComplementaryFilter(const ComplementaryFilter& orig);
  virtual ~ComplementaryFilter();
  
  /**
   * Calculate the height from the filtered data and put that in the complemantary DataStore object
   */
  void CalculateRealHeight();

  
 private:
  DataStore * const m_filtered_data;
  DataStore * const m_complementary_data;
};
}
}
#endif	/* COMPLEMENTARYFILTER_H */

