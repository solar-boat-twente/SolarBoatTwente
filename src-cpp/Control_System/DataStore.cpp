/* 
 * File:   DataStore.cpp
 */

#include <iostream>
#include "DataStore.h"

using namespace std;

DataStore::DataStore( ) {
     //cout << "Called constructor DataStore()" << endl;
}

DataStore::~DataStore( ) {
      //cout << "Called destructor DataStore()" << endl;
}
DataStore::XsensData& DataStore::GetXsensData( ) {
     //cout << "Called DataStore::GetXsensData()" << endl;
    // Return inhoud member variabele.
    return xsens_data;
}
DataStore::sensor_struct& DataStore::GetSensorData( ) {
     //cout << "Called DataStore::GetSensorData()" << endl;
    // Return inhoud member variabele.
    return sensor_data;
}
void DataStore::PutXsensData(XsensData * const data7) {
     //cout << "Called DataStore::PutXsensData()" << endl;
    // De inhoud van waar de pointer naar wijst toekennen aan member
    // variabele xsens_data.
    xsens_data = *data7;
}
void DataStore::PutSensorData(sensor_struct * const data) {
    // cout << "Called DataStore::PutXsensData()" << endl;
    // De inhoud van waar de pointer naar wijst toekennen aan member
    // variabele xsens_data.
    sensor_data = *data;
}

void DataStore::PutFilteredData(FilteredData * const data3) {
     //cout << "Called DataStore::PutFilteredData()" << endl;
    // De inhoud van waar de pointer naar wijst toekennen aan member
    // variabele xsens_data.
    Filtered_data = *data3;
}
DataStore::FilteredData& DataStore::GetFilteredData( ) {
    //cout << "Called DataStore::GetFilterData()" << endl;
    // Return inhoud member variabele.
    return Filtered_data;
}
void DataStore::PutRealData(RealData * const data4) {
     //cout << "Called DataStore::PutComplementaryData()" << endl;
    // De inhoud van waar de pointer naar wijst toekennen aan member
    // variabele xsens_data.
    Complementary_data = *data4;
}
DataStore::RealData& DataStore::GetRealData() {
    // cout << "Called DataStore::GetComplementaryData()" << endl;
    // Return inhoud member variabele.
    return Complementary_data;
}
 void DataStore::PutPIDSplitData(PIDDataSplit * const data){
   PID_split_data = *data;
 }

DataStore::PIDDataSplit& DataStore::GetPIDSplitData()  {
  return PID_split_data;
}

void DataStore::PutEposData(EposData * const data){
  epos_data = *data;
}


DataStore::EposData& DataStore::GetEposData()  {
  return epos_data;
}



void DataStore::PutPIDData(PIDDataTotal * const data5) {
     //cout << "Called DataStore::PutPIDDataTotalData()" << endl;
    // De inhoud van waar de pointer naar wijst toekennen aan member
    // variabele xsens_data.
    PID_data = *data5;
}

DataStore::PIDDataTotal&  DataStore::GetPIDData() {
    //cout << "Called DataStore::GetXsensData()" << endl;
    // Return inhoud member variabele.
    return PID_data;
}
void DataStore::PutWingData(AngleWings * const data6) {
     //cout << "Called DataStore::PutWingData()" << endl;
    // De inhoud van waar de pointer naar wijst toekennen aan member
    // variabele xsens_data.
    Wing_data = *data6;
}
DataStore::AngleWings&  DataStore::GetWingData() {
    //cout << "Called DataStore::GetWingData()" << endl;
    // Return inhoud member variabele.
    return Wing_data;
}
