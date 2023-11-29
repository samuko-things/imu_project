#ifndef GLOBAL_EEPROM_VARIABLES
#define GLOBAL_EEPROM_VARIABLES

#include <MatVectLab.h>
#include "low_pass_filter_setup.h"


///////////////////////////////////////////////////
float axRaw = 0.00;
float ayRaw = 0.00;
float azRaw = 0.00;

float axCal = 0.00;
float ayCal = 0.00;
float azCal = 0.00;

float axOff = 0.00;
float ayOff = 0.00;
float azOff = 0.00;

float acc_vect[3];

float R_mat[3][3];

float lin_acc_vect[3];
float lin_acc_x = 0.00;
float lin_acc_y = 0.00;
float lin_acc_z = 0.00;

float acc_gravity_vect[3] = {0.00, 0.00, -9.8};
////////////////////////////////////////////////////




////////////////////////////////////////////////////
float gxRaw = 0.00;
float gyRaw = 0.00;
float gzRaw = 0.00;

float gxCal = 0.00;
float gyCal = 0.00;
float gzCal = 0.00;

float gxOff = 0.00;
float gyOff = 0.00;
float gzOff = 0.00;
////////////////////////////////////////////////////




////////////////////////////////////////////////////
// magCal = A*(magRaw - b) using the A matrix and b vector to remove the magnetic offsets
float mxRaw = 0.00;
float myRaw = 0.00;
float mzRaw = 0.00;

float mxCal = 0.00;
float myCal = 0.00;
float mzCal = 0.00;

float A_mat[3][3];

float b_vect[3];

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
unsigned long kSampleTime_ms = 50, kTime_ms;
float kSampleTime = (float)kSampleTime_ms/1000.00;


float roll_est = 0.0, roll_uncertainty = 100.0;
float roll_rate_variance = 0.00;
float roll_angle_variance = 0.00;

float old_roll = 0.00;

void rollKalmanFilter1D(float roll_rate_measurement, float roll_angle_measurement){
  if(abs(roll_angle_measurement-old_roll)> 5.00){
    roll_est = roll_angle_measurement;
  }

  //prediction
  roll_est = roll_est + (kSampleTime * roll_rate_measurement);
  roll_uncertainty = roll_uncertainty + ((kSampleTime*kSampleTime) * roll_rate_variance);

  //correction
  float kalman_gain = roll_uncertainty/(roll_uncertainty + roll_angle_variance);
  roll_est = roll_est + (kalman_gain * (roll_angle_measurement - roll_est));
  roll_uncertainty = (1-kalman_gain) * roll_uncertainty;

  old_roll = roll_angle_measurement;
}


float pitch_est = 0.0, pitch_uncertainty = 100.0; 
float pitch_rate_variance = 0.00;
float pitch_angle_variance = 0.00;

float old_pitch=0.00;

void pitchKalmanFilter1D(float pitch_rate_measurement, float pitch_angle_measurement){
  if(abs(pitch_angle_measurement-old_pitch)> 5.00){
    pitch_est = pitch_angle_measurement;
  }

  //prediction
  pitch_est = pitch_est + (kSampleTime * pitch_rate_measurement);
  pitch_uncertainty = pitch_uncertainty + ((kSampleTime*kSampleTime) * pitch_rate_variance);

  //correction
  float kalman_gain = pitch_uncertainty/(pitch_uncertainty + pitch_angle_variance);
  pitch_est = pitch_est + (kalman_gain * (pitch_angle_measurement - pitch_est));
  pitch_uncertainty = (1-kalman_gain) * pitch_uncertainty;

  old_pitch = pitch_angle_measurement;
}


float yaw_est = 0.0, yaw_uncertainty = 10000.0;
float yaw_rate_variance = 0.00;
float yaw_angle_variance = 0.00;

float old_yaw=0.00;

void yawKalmanFilter1D(float yaw_rate_measurement, float yaw_angle_measurement){
  if(abs(yaw_angle_measurement-old_yaw)> 5.00){
    yaw_est = yaw_angle_measurement;
  }

  //prediction
  yaw_est = yaw_est + (kSampleTime * yaw_rate_measurement);
  yaw_uncertainty = yaw_uncertainty + ((kSampleTime*kSampleTime) * yaw_rate_variance);

  //correction
  float kalman_gain = yaw_uncertainty/(yaw_uncertainty + yaw_angle_variance);
  yaw_est = yaw_est + (kalman_gain * (yaw_angle_measurement - yaw_est));
  yaw_uncertainty = (1-kalman_gain) * yaw_uncertainty;

  old_yaw = yaw_angle_measurement;
}
////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////
float qw, qx, qy, qz;
////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////
float lin_acc_x_est = 0.0, lin_acc_x_uncertainty = 100.0;
float lin_acc_x_variance = 0.00144406228559;

void linAccxKalmanFilter1D(float lin_acc_x_measurement){
  //prediction
  lin_acc_x_est = lin_acc_x_est;
  lin_acc_x_uncertainty = lin_acc_x_uncertainty;

  //correction
  float kalman_gain = lin_acc_x_uncertainty/(lin_acc_x_uncertainty + lin_acc_x_variance);
  lin_acc_x_est = lin_acc_x_est + (kalman_gain * (lin_acc_x_measurement - lin_acc_x_est));
  lin_acc_x_uncertainty = (1-kalman_gain) * lin_acc_x_uncertainty;
}


float lin_acc_y_est = 0.0, lin_acc_y_uncertainty = 100.0;
float lin_acc_y_variance = 0.00070666749879;

void linAccyKalmanFilter1D(float lin_acc_y_measurement){
  //prediction
  lin_acc_y_est = lin_acc_y_est;
  lin_acc_y_uncertainty = lin_acc_y_uncertainty;

  //correction
  float kalman_gain = lin_acc_y_uncertainty/(lin_acc_y_uncertainty + lin_acc_y_variance);
  lin_acc_y_est = lin_acc_y_est + (kalman_gain * (lin_acc_y_measurement - lin_acc_y_est));
  lin_acc_y_uncertainty = (1-kalman_gain) * lin_acc_y_uncertainty;
}


float lin_acc_z_est = 0.0, lin_acc_z_uncertainty = 100.0;
float lin_acc_z_variance = 0.0054157543819900004;

void linAcczKalmanFilter1D(float lin_acc_z_measurement){
  //prediction
  lin_acc_z_est = lin_acc_z_est;
  lin_acc_z_uncertainty = lin_acc_z_uncertainty;

  //correction
  float kalman_gain = lin_acc_z_uncertainty/(lin_acc_z_uncertainty + lin_acc_z_variance);
  lin_acc_z_est = lin_acc_z_est + (kalman_gain * (lin_acc_z_measurement - lin_acc_z_est));
  lin_acc_z_uncertainty = (1-kalman_gain) * lin_acc_z_uncertainty;
}

////////////////////////////////////////////////////////////////////////////


// adaptive lowpass Filter
int order = 1;
float cutOffFreq = 0.5;

// Filter instance
AdaptiveLowPassFilter linAccxFilter(order, cutOffFreq);
AdaptiveLowPassFilter linAccyFilter(order, cutOffFreq);
AdaptiveLowPassFilter linAcczFilter(order, cutOffFreq);

#endif