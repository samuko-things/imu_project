/*
 * MODIFIED BY OBIAGBA SAMUEL
 * 
 * Copyright (c) 2023 Samuko Robotics Inc
 * 
 * 
 * 
 * Brian R Taylor
 * brian.taylor@bolderflight.com
 *
 * Copyright (c) 2021 Bolder Flight Systems Inc
 *
 */

#include <MatVectLab.h>
#include "mpu9250.h"

#include "serial_i2c_comm_api.h"



// /* Mpu9250 object, SPI bus, CS on pin 10 */
// bfs::Mpu9250 imu(&SPI, 10);

/* Mpu9250 object */
bfs::Mpu9250 imu;

unsigned long serialCommTime, serialCommSampleTime = 10;  // ms -> (1000/sampleTime) hz
unsigned long readImuTime, readImuSampleTime = 5; // ms -> (1000/sampleTime) hz


void setup() {
  initLed();
  offLed();

  /* Serial to display data */
  Serial.begin(115200);
  Serial.setTimeout(2);

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
    // Serial.println("Error initializing communication with IMU");
    
    while (1) {
      // delay(100);
      // onLed();
      // delay(100);
      // offLed();
    }
  }

  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    // Serial.println("Error configured SRD");
    while (1) {
      // delay(100);
      // onLed();
      // delay(100);
      // offLed();
    }
  }
  //----------------------------------------------------------------//


  // update global params with eeprom contents
  updateGlobalParamsFromEERPOM();
  /////////////////////////////////////////////

  delay(500);
  onLed();
  delay(1000);
  offLed();
  delay(500);
  onLed();
  delay(1000);
  offLed();


  serialCommTime = millis();
  readImuTime = millis();
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

      // convert NED frame to NWU frame
      axRaw = axRaw*1.00;
      ayRaw = ayRaw*-1.00;
      azRaw = azRaw*-1.00;

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

      // convert NED frame to NWU frame
      gxRaw = gxRaw*1.00;
      gyRaw = gyRaw*-1.00;
      gzRaw = gzRaw*-1.00;

      gxCal = gxRaw - gxOff;
      gyCal = gyRaw - gyOff;
      gzCal = gzRaw - gzOff;
      //--------------------------//


      //--------------------------//
      mxRaw = imu.mag_x_ut();
      myRaw = imu.mag_y_ut();
      mzRaw = imu.mag_z_ut();

      // convert NED frame to NWU frame
      mxRaw = mxRaw*1.00;
      myRaw = myRaw*-1.00;
      mzRaw = mzRaw*-1.00;

      // magCal = A_1*(magRaw - b) using the A matrix and b vector to remove the magnetic offsets
      mag_vect[0] = mxRaw;
      mag_vect[1] = myRaw;
      mag_vect[2] = mzRaw;
    
      vectOp.subtract(mag_vect, mag_vect, b_vect);  // mag_vect = mag_vect - b_vect
      vectOp.transform(mag_vect, A_mat, mag_vect);  // mag_vect = A_mat * mag_vect

      mxCal = mag_vect[0];
      myCal = mag_vect[1];
      mzCal = mag_vect[2];

      mag_vect[0] = mxCal;
      mag_vect[1] = myCal;
      mag_vect[2] = mzCal;
      //-------------------------//




      //---- COMPUTE ROLL PITCH & YAW -----------//
      // convert calibrated acc and mag vector to unit vector by normalizing
      vectOp.normalize(acc_vect_norm, acc_vect);
      vectOp.normalize(mag_vect_norm, mag_vect);


      // roll output is between -180 to 0 to +180
      roll_deg = atan2( acc_vect_norm[1], acc_vect_norm[2] ) * 180.0 / PI;

      // pitch angle will be between -180 to 0 to +180
      pitch_deg = atan2( (-1 * acc_vect_norm[0]), sqrt((acc_vect_norm[1] * acc_vect_norm[1]) + (acc_vect_norm[2] * acc_vect_norm[2])) ) * 180.0 / PI;
      if ((acc_vect_norm[0] < 0) && (acc_vect_norm[2] < 0)) {
        pitch_deg = 180.0 - pitch_deg;
      }
      else if ((acc_vect_norm[0] > 0) && (acc_vect_norm[2] < 0)) {
        pitch_deg = -180.0 - pitch_deg;
      }

      vectOp.copy(up, acc_vect_norm);
      vectOp.cross(west, up, mag_vect_norm);
      vectOp.cross(north, west, up);


      float Rroll_1[3][3] = {
        {1, 0, 0},
        {0, cos(radians(roll_deg)), (-1 * sin(radians(roll_deg)))},
        {0, sin(radians(roll_deg)), cos(radians(roll_deg))},
      };

      float Rpitch_1[3][3] = {
        {cos(radians(pitch_deg)), 0, sin(radians(pitch_deg))},
        {0, 1, 0},
        {(-1 * sin(radians(pitch_deg))), 0, cos(radians(pitch_deg))},
      };

      float n_vect[3];

      vectOp.transform(n_vect, Rroll_1, north);
      vectOp.transform(northCorrect, Rpitch_1, n_vect);

      // yaw angle will be between -180 to 0 to +180
      yaw_deg = atan2(-1 * northCorrect[1], northCorrect[0]) * 180.0 / PI;

      //-----------------------------------------------------------------------------//


      //-----------------------//
      roll = roll_deg * PI / 180.0;
      pitch = pitch_deg * PI / 180.0;
      yaw = yaw_deg * PI / 180.0;
      //-----------------------//


      //-----------------------//
      roll_rate = gxCal;
      pitch_rate = gyCal;
      yaw_rate = gzCal;
      //-----------------------//
    }


    readImuTime = millis();
  }

}