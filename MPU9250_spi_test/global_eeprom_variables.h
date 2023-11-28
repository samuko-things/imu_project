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

float axOff = 0.00;
float ayOff = 0.00;
float azOff = 0.00;

float acc_vect[3];
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
unsigned long kSampleTime_us = 100000, kTime_us;
float kSampleTime = (float)kSampleTime_us/1000000.00;


float roll_est = 0.0, roll_uncertainty = 100.0;
float roll_rate_variance = 0.00;
float roll_angle_variance = 0.00;

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
float pitch_rate_variance = 0.00;
float pitch_angle_variance = 0.00;

void pitchKalmanFilter1D(float pitch_rate_measurement, float pitch_angle_measurement){
  //prediction
  pitch_est = pitch_est + (kSampleTime * pitch_rate_measurement);
  pitch_uncertainty = pitch_uncertainty + ((kSampleTime*kSampleTime) * pitch_rate_variance);

  //correction
  float kalman_gain = pitch_uncertainty/(pitch_uncertainty + pitch_angle_variance);
  pitch_est = pitch_est + (kalman_gain * (pitch_angle_measurement - pitch_est));
  pitch_uncertainty = (1-kalman_gain) * pitch_uncertainty;
}


float yaw_est = 0.0, yaw_uncertainty = 10000.0;
float yaw_rate_variance = 0.00;
float yaw_angle_variance = 0.00;

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




/////////////////////////////////////////////////////////////////////////////
int wait_count = 0;



float new_yaw_deg=0.0, old_yaw_deg=0.0, inc_yaw_deg=0.0, init_inc_yaw_deg;
bool set_yaw_inc = false, start_yaw_inc = false;

void increment_yaw_angle(float yaw_deg) {
  new_yaw_deg = yaw_deg;
  if ((new_yaw_deg - old_yaw_deg) > 180) {
    inc_yaw_deg -= ((new_yaw_deg - old_yaw_deg) - 360);
  }
  else if ((new_yaw_deg - old_yaw_deg) < (-1 * 180)) {
    inc_yaw_deg -= ((new_yaw_deg - old_yaw_deg) + 360);
  }
  else {
    inc_yaw_deg += (new_yaw_deg - old_yaw_deg);
  }
  old_yaw_deg = new_yaw_deg;
}

void compute_yaw_increment(float yaw_deg) {
  if (!set_yaw_inc) {
    inc_yaw_deg = init_inc_yaw_deg;
    set_yaw_inc = true;
  }

  if (!start_yaw_inc) {
    old_yaw_deg = yaw_deg;
    start_yaw_inc = true;
  }

  increment_yaw_angle(yaw_deg);
}



float new_pitch_deg=0.0, old_pitch_deg=0.0, inc_pitch_deg=0.0, init_inc_pitch_deg;
bool set_pitch_inc = false, start_pitch_inc = false;

void increment_pitch_angle(float pitch_deg) {
  new_pitch_deg = pitch_deg;
  if ((new_pitch_deg - old_pitch_deg) > 180) {
    inc_pitch_deg -= ((new_pitch_deg - old_pitch_deg) - 360);
  }
  else if ((new_pitch_deg - old_pitch_deg) < (-1 * 180)) {
    inc_pitch_deg -= ((new_pitch_deg - old_pitch_deg) + 360);
  }
  else {
    inc_pitch_deg += (new_pitch_deg - old_pitch_deg);
  }
  old_pitch_deg = new_pitch_deg;
}

void compute_pitch_increment(float pitch_deg) {
  if (!set_pitch_inc) {
    inc_pitch_deg = init_inc_pitch_deg;
    set_pitch_inc = true;
  }

  if (!start_pitch_inc) {
    old_pitch_deg = pitch_deg;
    start_pitch_inc = true;
  }

  increment_pitch_angle(pitch_deg);
}




float new_roll_deg=0.0, old_roll_deg=0.0, inc_roll_deg=0.0, init_inc_roll_deg;
bool set_roll_inc = false, start_roll_inc = false;

void increment_roll_angle(float roll_deg) {
  new_roll_deg = roll_deg;
  if ((new_roll_deg - old_roll_deg) > 180) {
    inc_roll_deg -= ((new_roll_deg - old_roll_deg) - 360);
  }
  else if ((new_roll_deg - old_roll_deg) < (-1 * 180)) {
    inc_roll_deg -= ((new_roll_deg - old_roll_deg) + 360);
  }
  else {
    inc_roll_deg += (new_roll_deg - old_roll_deg);
  }
  old_roll_deg = new_roll_deg;
}

void compute_roll_increment(float roll_deg) {
  if (!set_roll_inc) {
    inc_roll_deg = init_inc_roll_deg;
    set_roll_inc = true;
  }

  if (!start_roll_inc) {
    old_roll_deg = roll_deg;
    start_roll_inc = true;
  }

  increment_roll_angle(roll_deg);
}

#endif


