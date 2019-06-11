/* 
 * File:   RuwDataFilter.h
 */

#ifndef _RUWDATAFILTER_H_
#define _RUWDATAFILTER_H_

#include <iostream>
#include <fstream>

#include "Biquad.h" 
#include "../DataStore.h"

using namespace std;

namespace MIO{
namespace Control{


class RawDataFilter
{
    public:
        RawDataFilter(double Fc = 20, double sample_rate = 80);
        virtual ~RawDataFilter();
        void add_data(DataStore * raw_state_data, DataStore * filtered_data){
          raw_state_data_ = raw_state_data;
          filtered_data_ = filtered_data;
        }
        void filter_data( );

    private:
        Biquad * low_pass_filter;
        DataStore *raw_state_data_;
        DataStore *filtered_data_;
        
        DataStore::FilteredData * m_filtered_data_;
};
}
}
#endif