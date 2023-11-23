#ifndef GLOBAL_EEPROM_VARIABLES
#define GLOBAL_EEPROM_VARIABLES

#include <MatVectLab.h>


///////////////////////////////////////////////////
float axRaw = 0.00;
float ayRaw = 0.00;
float azRaw = 0.00;

float axCal = 0.00;
float ayCal = 0.00;
float azCal = 0.00;

float axOff = 0.15544973333333273;
float ayOff = 0.302868866666667;
float azOff = -0.02610953333331878;

float acc_vect[3];
////////////////////////////////////////////////////




////////////////////////////////////////////////////
float gxRaw = 0.00;
float gyRaw = 0.00;
float gzRaw = 0.00;

float gxCal = 0.00;
float gyCal = 0.00;
float gzCal = 0.00;

float gxOff = -0.0005500000000000001;
float gyOff = -0.05275;
float gzOff = 0.01865;
////////////////////////////////////////////////////




////////////////////////////////////////////////////
// magCal = A*(magRaw - b) using the A matrix and b vector to remove the magnetic offsets
float mxRaw = 0.00;
float myRaw = 0.00;
float mzRaw = 0.00;

float mxCal = 0.00;
float myCal = 0.00;
float mzCal = 0.00;

float A_mat[3][3] = {
  {0.03124515, -0.00061432, 0.0007664},
  {-0.00061432, 0.0375395, -0.00094489},
  {0.0007664, -0.00094489, 0.02960192}
};

float b_vect[3] = {15.01861878, 35.19811548, -44.43469638};

float mag_vect[3];
////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
float acc_vect_norm_up[3], acc_vect_norm_down[3], mag_vect_norm[3];

float roll, pitch, yaw;
float roll_deg, pitch_deg, yaw_deg;
float roll_rate, pitch_rate, yaw_rate;
float north[3], east[3], down[3];
float northCorrect[3];
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
unsigned long kSampleTime_ms = 10, kTime_ms;
float kSampleTime = (float)kSampleTime_ms/1000.00;


float roll_est = 0.0, roll_uncertainty = 100.0;
float roll_rate_variance = 5.078905600000001e-07;
float roll_angle_variance = 6.410608640000001e-06;

void rollKalmanFilter1D(float roll_rate_measurement, float roll_angle_measurement){
  //prediction
  roll_est = roll_est + (kSampleTime * roll_rate_measurement);
  roll_uncertainty = roll_uncertainty + ((kSampleTime*kSampleTime) * roll_rate_variance);

  //correction
  float kalman_gain = roll_uncertainty/(roll_uncertainty + roll_angle_variance);
  roll_est = roll_est + (kalman_gain * (roll_angle_measurement - roll_est));
  roll_uncertainty = (1-kalman_gain) * roll_uncertainty;
}


float pitch_est = 0.0, pitch_uncertainty = 100.0; 
float pitch_rate_variance = 7.51696e-07;
float pitch_angle_variance = 6.329787639999999e-06;

void pitchKalmanFilter1D(float pitch_rate_measurement, float pitch_angle_measurement){
  //prediction
  pitch_est = pitch_est + (kSampleTime * pitch_rate_measurement);
  pitch_uncertainty = pitch_uncertainty + ((kSampleTime*kSampleTime) * pitch_rate_variance);

  //correction
  float kalman_gain = pitch_uncertainty/(pitch_uncertainty + pitch_angle_variance);
  pitch_est = pitch_est + (kalman_gain * (pitch_angle_measurement - pitch_est));
  pitch_uncertainty = (1-kalman_gain) * pitch_uncertainty;
}


float yaw_est = 0.0, yaw_uncertainty = 100.0;
float yaw_rate_variance = 9.06364e-07;
float yaw_angle_variance = 0.00077800094156;

void yawKalmanFilter1D(float yaw_rate_measurement, float yaw_angle_measurement){
  //prediction
  yaw_est = yaw_est + (kSampleTime * yaw_rate_measurement);
  yaw_uncertainty = yaw_uncertainty + ((kSampleTime*kSampleTime) * yaw_rate_variance);

  //correction
  float kalman_gain = yaw_uncertainty/(yaw_uncertainty + yaw_angle_variance);
  yaw_est = yaw_est + (kalman_gain * (yaw_angle_measurement - yaw_est));
  yaw_uncertainty = (1-kalman_gain) * yaw_uncertainty;
}
////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////
float qw, qx, qy, qz;
////////////////////////////////////////////////////////////////////////////

#endif


