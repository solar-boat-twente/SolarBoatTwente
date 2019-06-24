/* 
 * File:   Force_to_wing_angle.h
 * Author: User
 *
 * Created on 28 februari 2019, 10:05
 */

#ifndef FORCE_TO_WING_ANGLE_H
#define	FORCE_TO_WING_ANGLE_H

#include "../DataStore.h"
#include "../../../solarboattwente.h"

namespace MIO{
namespace control{

constexpr float kMinSpeedHeight = 5.2;
constexpr float kMinSpeedRoll = 2.7;
constexpr float kMinHeight = 0.4;
constexpr int kLiftOfForce = 1000;

constexpr bool kControlTesting = false;
constexpr int kControlTestVelocity = 4;

constexpr int kRpmToMeterPerSecond = 216;
constexpr bool kUseXsensVelocity = false;


class ForceToWingAngle {
 public:

    ForceToWingAngle(DataStore * const control_data);    
    
    void MMA(structures::PowerInput * power_input);   
    
    void MMA(float velocity);
    
    void set_velocity(float velocity);
 private:
  void calculate_inverse_matrix();
  
  /**
   * Compute the lift coefficient from the force and the area, takes the velocity which is a
   * member variable and a constant density defined in the class
   * @param force Force which the wing should give
   * @param area Area of the wing
   * @return Lift coefficient alpha in units ???
   */
  float compute_lift_coefficient_(float force, float area);
  
  /**
   * Compute the force based on the required pitch, roll and height forces found in
   * the PIDDataTotal structure from data store. 
   * @param pid_forces A PIDDataTotal structure containing all the forces according to the pid
   * @param row_inverse_matrix Row from the inverse matrix which should be used to compute the force
   * @return Force required using the inverse MMA matrix and force in N.
   */
  float compute_force_(float force_height, float force_pitch, float force_roll, float row_inverse_matrix[]);
  
  /**
   * Calculate the angle the wing should have based on the lift_coefficient, uses a 
   * constant to go from lift coefficient to angle. In the 2d world this should be 2 Pi.
   * However in the 3d space this constant is a bit arbitrary and therefore defined as 5
   * @param lift_coefficient The lift coefficient you would like to get the angle for
   * @return Angle the wing should have in degrees.
   */
  float compute_wing_angle_(float lift_coefficient, int lift_to_slope = kLiftSlope);
  
  /**
   * Compute the real angle of the wings taking into account the zero lift angle and the pitch correction
   * @param original_angle The angle the wing should have
   * @param pitch_correction The current pitch of the boat which the wing should be corrected for
   * @param zero_lift_correction The angle at which there is 0 lift
   * @return 
   */
  float compute_real_angle_(float original_angle, float pitch_correction, float zero_lift_correction = kZeroLiftAngle);
  
  
  
  //MMA to go from forces (force height, moment pitch and moment roll to force 
  //wing left, force wing right, force wing back:
  /**
   * F_height = F_left + F_right + F_back
   * 
   * M_pitch  = 1.54 * F_left + 1.54 * F_right - 2.83 * F_back
   * 
   * M_Roll   = -0.37 * F_left + 0.37 * F_right
   */
  const float matrix_MMA[3][3] = {
    {1, 1, 1},
    {1.54, 1.54, -2.83},
    {-0.37, 0.37, 0},
  }; 
  
  DataStore * const control_data_;
  
  float inverse_matrix_MMA_[3][3];
  
  static constexpr float kLeftSurface = 0.05246193;   //m2
  static constexpr float kRightSurface = 0.05246193;       
  static constexpr float kBackSurface = 0.058134;
  static constexpr float kDensity = 1025; //kg/m3 density salt water
  static constexpr int kLiftSlope = 5;
  static constexpr float kZeroLiftAngle = -0.08;//0.08; //-0.05236;      //radians
    
  float velocity_ = 0;
};   
}
}
#endif
