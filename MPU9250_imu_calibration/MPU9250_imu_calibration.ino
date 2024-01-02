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

unsigned long serialCommTime, serialCommSampleTime = 10; // ms -> (1000/sampleTime) hz
unsigned long readImuTime, readImuSampleTime = 5;        // ms -> (1000/sampleTime) hz

void setup()
{
  initLed();
  offLed();

  /* Serial to display data */
  Serial.begin(115200);
  Serial.setTimeout(2);

  //---------------- START IMU IN SPI MODE -----------------------//
  /* Start the SPI bus */
  SPI.begin();
  //----------------------------------------------------------------//

  //---------------- START IMU IN I2C MODE -----------------------//
  // /* Start the I2C bus */
  // Wire.begin();
  // Wire.setClock(400000);
  // /* I2C bus,  0x68 address */
  // imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  //----------------------------------------------------------------//

  //---------------- INITIALIZE IMU -----------------------//
  /* Initialize and configure IMU */
  if (!imu.Begin())
  {
    // Serial.println("Error initializing communication with IMU");

    while (1)
    {
      // delay(100);
      // onLed();
      // delay(100);
      // offLed();
    }
  }

  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19))
  {
    // Serial.println("Error configured SRD");
    while (1)
    {
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

void loop()
{

  ////////// using the serial communiaction API ////////////////////////
  if ((millis() - serialCommTime) >= serialCommSampleTime)
  {
    serialReceiveAndSendData();
    serialCommTime = millis();
  }
  //////////////////////////////////////////////////////////////////////

  if ((millis() - readImuTime) >= readImuSampleTime)
  {

    if (imu.Read())
    {
      //------------READ ACC DATA AND CALIBRATE---------------//
      axRaw = imu.accel_x_mps2();
      ayRaw = imu.accel_y_mps2();
      azRaw = imu.accel_z_mps2();

      // convert NED frame to NWU frame
      axRaw = axRaw * 1.00;
      ayRaw = ayRaw * -1.00;
      azRaw = azRaw * -1.00;

      axCal = axRaw - axOff;
      ayCal = ayRaw - ayOff;
      azCal = azRaw - azOff;

      acc_vect[0] = axCal;
      acc_vect[1] = ayCal;
      acc_vect[2] = azCal;
      //------------------------------------------------------//

      //-----------READ GYRO DATA AND CALIBRATE---------------//
      gxRaw = imu.gyro_x_radps();
      gyRaw = imu.gyro_y_radps();
      gzRaw = imu.gyro_z_radps();

      // convert NED frame to NWU frame
      gxRaw = gxRaw * 1.00;
      gyRaw = gyRaw * -1.00;
      gzRaw = gzRaw * -1.00;

      gxCal = gxRaw - gxOff;
      gyCal = gyRaw - gyOff;
      gzCal = gzRaw - gzOff;
      //-----------------------------------------------------//

      //-----------READ MAG DATA AND CALIBRATE---------------//
      mxRaw = imu.mag_x_ut();
      myRaw = imu.mag_y_ut();
      mzRaw = imu.mag_z_ut();

      // convert NED frame to NWU frame
      mxRaw = mxRaw * 1.00;
      myRaw = myRaw * -1.00;
      mzRaw = mzRaw * -1.00;

      // magCal = A_1*(magRaw - b) using the A matrix and b vector to remove the magnetic offsets
      mag_vect[0] = mxRaw;
      mag_vect[1] = myRaw;
      mag_vect[2] = mzRaw;

      vectOp.subtract(mag_vect, mag_vect, b_vect); // mag_vect = mag_vect - b_vect
      vectOp.transform(mag_vect, A_mat, mag_vect); // mag_vect = A_mat * mag_vect

      mxCal = mag_vect[0];
      myCal = mag_vect[1];
      mzCal = mag_vect[2];

      mag_vect[0] = mxCal;
      mag_vect[1] = myCal;
      mag_vect[2] = mzCal;
      //---------------------------------------------------//

      //---- GENERATE NED AND DCM -----------//
      // convert calibrated acc and mag vector to unit vector by normalizing
      vectOp.normalize(acc_vect_norm, acc_vect);
      vectOp.normalize(mag_vect_norm, mag_vect);

      vectOp.copy(u, acc_vect_norm);     // up = acc_vect_norm
      vectOp.cross(w, u, mag_vect_norm); // west = up (cross) mag_vect_norm
      vectOp.cross(n, w, u);             // north = west (cross) up

      // generate DCM
      float DCM[3][3];

      DCM[0][0] = n[0];
      DCM[0][1] = w[0];
      DCM[0][2] = u[0];

      DCM[1][0] = n[1];
      DCM[1][1] = w[1];
      DCM[1][2] = u[1];

      DCM[2][0] = n[2];
      DCM[2][1] = w[2];
      DCM[2][2] = u[2];

      //-----------------------------------------------------//

      //------ CALC QUAT from DCM -----------------------//
      float q0_root = sqrt(1 + DCM[0][0] + DCM[1][1] + DCM[2][2]);
      float q1_root = sqrt(1 + DCM[0][0] - DCM[1][1] - DCM[2][2]);
      float q2_root = sqrt(1 - DCM[0][0] + DCM[1][1] - DCM[2][2]);
      float q3_root = sqrt(1 - DCM[0][0] - DCM[1][1] + DCM[2][2]);

      if ((DCM[1][1] > -DCM[2][2]) && (DCM[0][0] > -DCM[1][1]) && (DCM[0][0] > -DCM[2][2]))
      {
        qw = 0.5 * q0_root;
        qx = 0.5 * (DCM[1][2] - DCM[2][1]) / q0_root;
        qy = 0.5 * (DCM[2][0] - DCM[0][2]) / q0_root;
        qz = 0.5 * (DCM[0][1] - DCM[1][0]) / q0_root;
      }
      else if ((DCM[1][1] < -DCM[2][2]) && (DCM[0][0] > DCM[1][1]) && (DCM[0][0] > DCM[2][2]))
      {
        qw = 0.5 * (DCM[1][2] - DCM[2][1]) / q1_root;
        ;
        qx = 0.5 * q1_root;
        qy = 0.5 * (DCM[0][1] + DCM[1][0]) / q1_root;
        qz = 0.5 * (DCM[2][0] + DCM[0][2]) / q1_root;
      }
      else if ((DCM[1][1] > DCM[2][2]) && (DCM[0][0] < DCM[1][1]) && (DCM[0][0] < -DCM[2][2]))
      {
        qw = 0.5 * (DCM[2][0] - DCM[0][2]) / q2_root;
        qx = 0.5 * (DCM[0][1] + DCM[1][0]) / q2_root;
        qy = 0.5 * q2_root;
        qz = 0.5 * (DCM[1][2] + DCM[2][1]) / q2_root;
      }
      else if ((DCM[1][1] < DCM[2][2]) && (DCM[0][0] < -DCM[1][1]) && (DCM[0][0] < DCM[2][2]))
      {
        qw = 0.5 * (DCM[0][1] - DCM[1][0]) / q3_root;
        qx = 0.5 * (DCM[2][0] + DCM[0][2]) / q3_root;
        qy = 0.5 * (DCM[1][2] + DCM[2][1]) / q3_root;
        qz = 0.5 * q3_root;
      }
      //-------------------------------------------------//

      //------ CALC RPY from QUAT -----------------------//
      float t0 = 2.0 * (qw * qx + qy * qz);
      float t1 = 1.0 - 2.0 * (qx * qx + qy * qy);
      roll = atan2(t0, t1);

      float t2 = 2.0 * (qw * qy - qz * qx);
      if (t2 > 1.0)
      {
        t2 = 1.0;
      }
      else if (t2 < -1.0)
      {
        t2 = -1.0;
      }
      pitch = asin(t2);

      float t3 = 2.0 * (qw * qz + qx * qy);
      float t4 = 1.0 - 2.0 * (qy * qy + qz * qz);
      yaw = atan2(t3, t4);
      //----------------------------------------------------//


      //--------- GET RPY RATE----------------------//
      roll_rate = gxCal;
      pitch_rate = gyCal;
      yaw_rate = gzCal;
      //--------------------------------------------//
      

      readImuTime = millis();
    }
  }
}