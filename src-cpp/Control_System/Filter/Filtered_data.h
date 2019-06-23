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
namespace control{


class RawDataFilter
{
    public:
        RawDataFilter(DataStore * const control_data, double Fc = 20, double sample_rate = 80);
        virtual ~RawDataFilter();
        void filter_data( );

    private:
        Biquad * low_pass_filter;
        DataStore * const control_data_;
        
        DataStore::FilteredData * const filtered_data_;
};
}
}
#endif