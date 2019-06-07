/* 
 * File:   DataStore.h
 */

#ifndef _DATASTORE_H_
#define _DATASTORE_H_

//#include <string>

class DataStore {
  

    public:
        
        struct sensor_struct {
            float pitch;
            float roll;
            float Z_accel;
            float angle_left;
            float angle_right;
        };
        
        struct FilteredData {
          float filtered_pitch;
          float filtered_roll ;
          float filtered_Z_accel ;
          float filtered_angle_left ;
          float filtered_angle_right ;
        };
        
        struct RealData {
          float Real_height;
          float Real_roll;
          float Real_pitch;
        };
        
        struct PIDDataTotal {
          float Force_height;
          float Force_roll;
          float Force_pitch;
        };
        
        struct AngleWings {
            float Wing_left;
            float Wing_right;
            float Wing_back;
        };
        
        struct XsensData{
          float roll;
          float yaw;
          float pitch;
          float acceleration_x;
          float acceleration_y;
          float acceleration_z;
          float velocity_x;
          float velocity_y;
          float velocity_z;
          float latitude;
          float longitude;            
        };
        DataStore();
        
        virtual ~DataStore();
        
        void PutSensorData(sensor_struct * data);
        void PutFilteredData(FilteredData * data3);
        void PutComplementaryData(RealData * data4);
        void PutPIDData(PIDDataTotal * data5);
        void PutWingData(AngleWings * data6);
        void PutXsensData(XsensData * data7);
        sensor_struct GetSensorData();
        FilteredData GetFilteredData();
        RealData GetComplementaryData();
        PIDDataTotal GetPIDData();
        AngleWings GetWingData();
        XsensData GetXsensData();

    private:
        XsensData xsens_data;
        sensor_struct sensor_data;
        FilteredData Filtered_data;
        RealData Complementary_data;
        PIDDataTotal PID_data;
        AngleWings Wing_data;
};

#endif

