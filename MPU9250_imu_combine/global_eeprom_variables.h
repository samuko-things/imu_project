#ifndef GLOBAL_EEPROM_VARIABLES
#define GLOBAL_EEPROM_VARIABLES


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
float acc_vect_norm[3];

float linear_acc_vect[3];

float accx_variance = 0.00;
float accy_variance = 0.00;
float accz_variance = 0.00;
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
float mag_vect_norm[3];
////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
float roll, pitch, yaw;
float roll_rate, pitch_rate, yaw_rate;

// sensor body NWU frame
float n[3], w[3], u[3];
//////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////
float ref_yaw=0.00;
bool startYawRefAngle = true;
int count=0;
float heading_est=0.00;

float yaw_inc=0.0, prev_yaw=0.0, new_yaw=0.0;
bool set_yaw_inc=false, start_yaw_inc=false;
/////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
float roll_est = 0.000;
float roll_rate_bias_est = 0.00;

float P_roll[2][2]; // roll_est and bias uncertainty matrix
float S_roll = 0.00;
float K_roll[2]; // kalman gain for roll_est and roll_rate_bias_est respectively
float Q_roll = 0.00; // roll rate gyro bias process noise (i.e gyroX rate variance)
float R_roll = 0.00; // roll angle measurement variance

float old_roll = 0.00;
unsigned long rollLastTime = micros();

void rollKalmanFilter(float roll_rate_measurement, float roll_angle_measurement){
  if(abs(roll_angle_measurement-old_roll)> 2.00){
    roll_est = roll_angle_measurement;
  }

  float dt = (float)(micros() - rollLastTime)/1.0e6;

  //prediction
  roll_est = roll_est + (dt * (roll_rate_measurement - roll_rate_bias_est));

  P_roll[0][0] += dt * ((dt*P_roll[1][1]) - P_roll[0][1] - P_roll[1][0] + Q_roll);
  P_roll[0][1] -= dt * P_roll[1][1];
  P_roll[1][0] -= dt * P_roll[1][1];
  P_roll[1][1] += Q_roll * dt;

  //correction
  
  S_roll = P_roll[0][0]+R_roll;

  K_roll[0] = P_roll[0][0] / S_roll;
  K_roll[1] = P_roll[1][0] / S_roll;

  roll_est = roll_est + (K_roll[0] * (roll_angle_measurement - roll_est));
  roll_rate_bias_est = roll_rate_bias_est + (K_roll[1] * (roll_angle_measurement - roll_est));

  P_roll[0][0] -= K_roll[0] * P_roll[0][0];
  P_roll[0][1] -= K_roll[0] * P_roll[0][1];
  P_roll[1][0] -= K_roll[1] * P_roll[0][0];
  P_roll[1][1] -= K_roll[1] * P_roll[0][1];

  rollLastTime = micros();

  old_roll = roll_angle_measurement;
}
////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////
float pitch_est = 0.000;
float pitch_rate_bias_est = 0.00;

float P_pitch[2][2]; // pitch_est and bias uncertainty matrix(i.e gyroX rate variance)
float S_pitch = 0.00;
float K_pitch[2]; // kalman gain for pitch_est and pitch_rate_bias_est respectively
float Q_pitch = 0.00; // pitch rate gyro bias process noise (i.e gyroY rate variance)
float R_pitch = 0.00; // pitch angle measurement variance

float old_pitch=0.00;
unsigned long pitchLastTime = micros();

void pitchKalmanFilter(float pitch_rate_measurement, float pitch_angle_measurement){
  if(abs(pitch_angle_measurement-old_pitch)> 5.00){
    pitch_est = pitch_angle_measurement;
  }

  float dt = (float)(micros() - pitchLastTime)/1.0e6;

  //prediction
  pitch_est = pitch_est + (dt * (pitch_rate_measurement - pitch_rate_bias_est));

  P_pitch[0][0] += dt * ((dt*P_pitch[1][1]) - P_pitch[0][1] - P_pitch[1][0] + Q_pitch);
  P_pitch[0][1] -= dt * P_pitch[1][1];
  P_pitch[1][0] -= dt * P_pitch[1][1];
  P_pitch[1][1] += Q_pitch * dt;

  //correction
  S_pitch = P_pitch[0][0]+R_pitch;

  K_pitch[0] = P_pitch[0][0] / S_pitch;
  K_pitch[1] = P_pitch[1][0] / S_pitch;

  pitch_est = pitch_est + (K_pitch[0] * (pitch_angle_measurement - pitch_est));
  pitch_rate_bias_est = pitch_rate_bias_est + (K_pitch[1] * (pitch_angle_measurement - pitch_est));

  P_pitch[0][0] -= K_pitch[0] * P_pitch[0][0];
  P_pitch[0][1] -= K_pitch[0] * P_pitch[0][1];
  P_pitch[1][0] -= K_pitch[1] * P_pitch[0][0];
  P_pitch[1][1] -= K_pitch[1] * P_pitch[0][1];

  pitchLastTime = micros();

  old_pitch = pitch_angle_measurement;
}
////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////
float yaw_est = 0.000;
float yaw_rate_bias_est = 0.00;

float P_yaw[2][2]; // yaw_est and bias uncertainty matrix
float S_yaw = 0.00;
float K_yaw[2]; // kalman gain for yaw_est and yaw_rate_bias_est respectively
float Q_yaw = 0.00; // yaw rate gyro bias process noise (i.e gyroZ rate variance)
float R_yaw = 0.00; // yaw angle measurement variance

float old_yaw=0.00;
unsigned long yawLastTime = micros();

void yawKalmanFilter(float yaw_rate_measurement, float yaw_angle_measurement){
  if(abs(yaw_angle_measurement-old_yaw)> 5.00){
    yaw_est = yaw_angle_measurement;
  }

  float dt = (float)(micros() - yawLastTime)/1.0e6;

  //prediction
  yaw_est = yaw_est + (dt * (yaw_rate_measurement - yaw_rate_bias_est));

  P_yaw[0][0] += dt * ((dt*P_yaw[1][1]) - P_yaw[0][1] - P_yaw[1][0] + Q_yaw);
  P_yaw[0][1] -= dt * P_yaw[1][1];
  P_yaw[1][0] -= dt * P_yaw[1][1];
  P_yaw[1][1] += Q_yaw * dt;

  //correction
  S_yaw = P_yaw[0][0]+R_yaw;

  K_yaw[0] = P_yaw[0][0] / S_yaw;
  K_yaw[1] = P_yaw[1][0] / S_yaw;

  yaw_est = yaw_est + (K_yaw[0] * (yaw_angle_measurement - yaw_est));
  yaw_rate_bias_est = yaw_rate_bias_est + (K_yaw[1] * (yaw_angle_measurement - yaw_est));

  P_yaw[0][0] -= K_yaw[0] * P_yaw[0][0];
  P_yaw[0][1] -= K_yaw[0] * P_yaw[0][1];
  P_yaw[1][0] -= K_yaw[1] * P_yaw[0][0];
  P_yaw[1][1] -= K_yaw[1] * P_yaw[0][1];

  yawLastTime = micros();

  old_yaw = yaw_angle_measurement;
}
////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////
float qw=0.00;
float qx=0.00;
float qy=0.00;
float qz=0.00;
////////////////////////////////////////////////////////////////////////////


#endif


