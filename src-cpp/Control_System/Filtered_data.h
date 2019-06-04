/* 
 * File:   RuwDataFilter.h
 */

#ifndef _RUWDATAFILTER_H_
#define _RUWDATAFILTER_H_

#include <iostream>
#include <fstream>

#include "Biquad.h" 
#include "DataStore.h"
#include "Xsens.h"
using namespace std;


class RuwDataFilter
{
    public:
        RuwDataFilter();
        virtual ~RuwDataFilter();

        DataStore *m_ruwe_state_data;
        DataStore *m_filtered_data;
        
        void FilterIt( );

    private:
        Biquad *lpFilter;
};

#endif