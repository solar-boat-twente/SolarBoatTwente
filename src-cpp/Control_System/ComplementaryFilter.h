/* 
 * File:   ComplementaryFilter.h
 * Author: arjan
 *
 * Created on 1 maart 2019, 11:49
 */

#ifndef COMPLEMENTARYFILTER_H
#define	COMPLEMENTARYFILTER_H
#include "DataStore.h"
#include <stdio.h>      /* printf */
#include <math.h>       /* sin */
class ComplementaryFilter {
public:
    
    
    ComplementaryFilter();
    ComplementaryFilter(const ComplementaryFilter& orig);
    DataStore *m_filtered_data;
    DataStore *m_complementary_data;
    void CalculateRealHeight();
    virtual ~ComplementaryFilter();
private:

};

#endif	/* COMPLEMENTARYFILTER_H */

