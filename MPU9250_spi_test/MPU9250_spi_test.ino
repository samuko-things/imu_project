/*
 * MODIFIED BY OBIAGBA SAMUEL
 * Brian R Taylor
 * brian.taylor@bolderflight.com
 *
 * Copyright (c) 2021 Bolder Flight Systems Inc
 *
 */


#include "global_eeprom_variables.h"
#include "serial_i2c_comm_api.h"
#include "mpu9250.h"


// /* Mpu9250 object, SPI bus, CS on pin 10 */
// bfs::Mpu9250 imu(&SPI, 10);

/* Mpu9250 object */
bfs::Mpu9250 imu;

unsigned long serialCommTime, serialCommSampleTime = 10;  // ms -> (1000/sampleTime) hz
unsigned long readImuTime, readImuSampleTime = 5; // ms -> (1000/sampleTime) hz
unsigned long printTime_ms, printSampleTime_ms = 20; // ms -> (1000/sampleTime) hz


void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  Serial.setTimeout(2);

  // update global params with eeprom contents
  updateGlobalParamsFromEERPOM();
  /////////////////////////////////////////////

  delay(500);

  initLed();
  offLed();
  delay(500);
  onLed();
  delay(1000);
  offLed();


  //---------------- START IMU IN SPI MODE -----------------------//
  /* Start the SPI bus */
  // SPI.begin();
  //----------------------------------------------------------------//
  


  //---------------- START IMU IN I2C MODE -----------------------//
   /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);
  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  //----------------------------------------------------------------//



  //---------------- INITIALIZE IMU -----------------------//
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while (1) {
    }
  }

  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while (1) {
    }
  }
  //----------------------------------------------------------------//



  serialCommTime = millis();
  readImuTime = millis();
  printTime_ms = millis();

  kTime_us = micros();
  rollLastTime = micros();
  pitchLastTime = micros();
  yawLastTime = micros();
  
}







void loop() {

  ////////// using the serial communiaction API ////////////////////////
  if ((millis() - serialCommTime) >= serialCommSampleTime) {
    serialReceiveAndSendData();
    serialCommTime = millis();
  }
  //////////////////////////////////////////////////////////////////////


  if ((millis() - readImuTime) >= readImuSampleTime) {

    if (imu.Read()) {
      //---------------------------//
      axRaw = imu.accel_x_mps2();
      ayRaw = imu.accel_y_mps2();
      azRaw = imu.accel_z_mps2();

      axCal = axRaw - axOff;
      ayCal = ayRaw - ayOff;
      azCal = azRaw - azOff;

      acc_vect[0] = axCal;
      acc_vect[1] = ayCal;
      acc_vect[2] = azCal;
      //--------------------------//


      //--------------------------//
      gxRaw = imu.gyro_x_radps();
      gyRaw = imu.gyro_y_radps();
      gzRaw = imu.gyro_z_radps();

      gxCal = gxRaw - gxOff;
      gyCal = gyRaw - gyOff;
      gzCal = gzRaw - gzOff;
      //--------------------------//


      //--------------------------//
      mxRaw = imu.mag_x_ut();
      myRaw = imu.mag_y_ut();
      mzRaw = imu.mag_z_ut();

      // magCal = A_1*(magRaw - b) using the A matrix and b vector to remove the magnetic offsets
      mag_vect[0] = mxRaw;
      mag_vect[1] = myRaw;
      mag_vect[2] = mzRaw;
    
      vectOp.subtract(mag_vect, mag_vect, b_vect);  // mag_vect = mag_vect - b_vect
      vectOp.transform(mag_vect, A_mat, mag_vect);  // mag_vect = A_mat * mag_vect

      mxCal = mag_vect[0];
      myCal = mag_vect[1];
      mzCal = mag_vect[2];
      //-------------------------//




      //---- COMPUTE ROLL PITCH & YAW -----------//
      // convert calibrated acc and mag vector to unit vector by normalizing
      vectOp.normalize(acc_vect_norm_down, acc_vect);
      vectOp.normalize(mag_vect_norm, mag_vect);

      //acc_vect_norm_up = -1.00*acc_vect_norm_down
      vectOp.scale(acc_vect_norm_up, acc_vect_norm_down, -1.00); 

      // roll output is between -180 to 0 to +180
      roll_deg = atan2( acc_vect_norm_up[1], acc_vect_norm_up[2] ) * 180.0 / PI; //acc_y/acc_z

      // pitch angle will be between -180 to 0 to +180
      pitch_deg = atan2( (-1 * acc_vect_norm_up[0]), sqrt((acc_vect_norm_up[1] * acc_vect_norm_up[1]) + (acc_vect_norm_up[2] * acc_vect_norm_up[2])) ) * 180.0 / PI; //-acc_x/(sqrt(acc_y^2 + acc_z^2))
      if ((acc_vect_norm_up[0] < 0) && (acc_vect_norm_up[2] < 0)) {
        pitch_deg = 180.0 - pitch_deg;
      }
      else if ((acc_vect_norm_up[0] > 0) && (acc_vect_norm_up[2] < 0)) {
        pitch_deg = -180.0 - pitch_deg;
      }

      vectOp.copy(down, acc_vect_norm_down);
      vectOp.cross(east, down, mag_vect_norm);
      vectOp.cross(north, east, down);

      // // yaw angle will be between -180 to 0 to +180
      // yaw_deg = atan2(-1 * north[1], north[0]) * 180.0 / PI;

      float cr = cos(radians(roll_deg)), sr = sin(radians(roll_deg));
      float cp = cos(radians(pitch_deg)), sp = sin(radians(pitch_deg));

      float Rroll[3][3] = {
        {1, 0, 0},
        {0, cr, (-1 * sr)},
        {0, sr, cr},
      };

      float Rpitch[3][3] = {
        {cp, 0, sp},
        {0, 1, 0},
        {(-1 * sp), 0, cp},
      };

      float n_vect[3];

      vectOp.transform(n_vect, Rroll, north);
      vectOp.transform(northCorrect, Rpitch, n_vect);

      // yaw angle will be between -180 to 0 to +180
      yaw_deg = atan2(-1 * northCorrect[1], northCorrect[0]) * 180.0 / PI;

      //-------------------------------------------------------------------------------------//



      //-----------------------//
      roll_deg = 1.00 * roll_deg;
      pitch_deg = -1.00 * pitch_deg;
      yaw_deg = -1.00 * yaw_deg;
      //-----------------------//


      //-----------------------//
      roll = roll_deg * PI / 180.0;
      pitch = pitch_deg * PI / 180.0;
      yaw = yaw_deg * PI / 180.0;
      //-----------------------//


      //-----------------------//
      roll_rate = 1.00 * gxCal;
      pitch_rate = -1.00 * gyCal;
      yaw_rate = -1.00 * gzCal;
      //-----------------------//

    }

    //-------- APPLY 1D KALMAN FILTER TO ROLL, PITCH AND YAW ------------//
    rollKalmanFilter(roll_rate, roll);
    pitchKalmanFilter(pitch_rate, pitch);
    yawKalmanFilter(yaw_rate, yaw);
    //--------------------------------------------------------------------//

    //------- CONVERT FILTERED RPY TO QUATERNIONS -----------------------//
    qx = ( sin(roll_est/2) * cos(pitch_est/2) * cos(yaw_est/2) ) - ( cos(roll_est/2) * sin(pitch_est/2) * sin(yaw_est/2) );
    qy = ( cos(roll_est/2) * sin(pitch_est/2) * cos(yaw_est/2) ) + ( sin(roll_est/2) * cos(pitch_est/2) * sin(yaw_est/2) );
    qz = ( cos(roll_est/2) * cos(pitch_est/2) * sin(yaw_est/2) ) - ( sin(roll_est/2) * sin(pitch_est/2) * cos(yaw_est/2) );
    qw = ( cos(roll_est/2) * cos(pitch_est/2) * cos(yaw_est/2) ) + ( sin(roll_est/2) * sin(pitch_est/2) * sin(yaw_est/2) );
    // -------------------------------------------------------------------//

    readImuTime = millis();
  }

  // if ((millis() - kTime_us) >= kSampleTime_us) {

  //   //-------- APPLY 1D KALMAN FILTER TO ROLL, PITCH AND YAW ------------//
  //   rollKalmanFilter(roll_rate, roll);
  //   pitchKalmanFilter(pitch_rate, pitch);
  //   yawKalmanFilter(yaw_rate, yaw);
  //   //--------------------------------------------------------------------//

  //   //------- CONVERT FILTERED RPY TO QUATERNIONS -----------------------//
  //   qx = ( sin(roll_est/2) * cos(pitch_est/2) * cos(yaw_est/2) ) - ( cos(roll_est/2) * sin(pitch_est/2) * sin(yaw_est/2) );
  //   qy = ( cos(roll_est/2) * sin(pitch_est/2) * cos(yaw_est/2) ) + ( sin(roll_est/2) * cos(pitch_est/2) * sin(yaw_est/2) );
  //   qz = ( cos(roll_est/2) * cos(pitch_est/2) * sin(yaw_est/2) ) - ( sin(roll_est/2) * sin(pitch_est/2) * cos(yaw_est/2) );
  //   qw = ( cos(roll_est/2) * cos(pitch_est/2) * cos(yaw_est/2) ) + ( sin(roll_est/2) * sin(pitch_est/2) * sin(yaw_est/2) );
  //   // -------------------------------------------------------------------//

  //   kTime_us = micros();
  // }

}