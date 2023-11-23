#include <matrixlab.h>
#include <Wire.h>


class Imu {
  public:
    void init() {
      initIMU();
    }

    void setAccOffset(double x_offset, double y_offset, double z_offset) {
      accX_offset = x_offset;
      accY_offset = y_offset;
      accZ_offset = z_offset;
    }

    void setGyroOffset(double x_offset, double y_offset, double z_offset) {
      gyroX_offset = x_offset;
      gyroY_offset = y_offset;
      gyroZ_offset = z_offset;
    }

    void setMagOffset(double (&A_matrix)[3][3], double (&b_vector)[3][1]) {
      mat::copy(b, b_vector);
      mat::copy(A_1, A_matrix);
    }

    void computeRollPitchYaw() {
      ComputeRollPitchYaw();
    }

    double getRollAngle() {
      return roll;
    }

    double getPitchAngle() {
      return pitch;
    }

    double getYawAngle() {
      return yaw;
    }

    double getAbsoluteRoll() {
      return Roll;
    }

    double getAbsolutePitch() {
      return Pitch;
    }

    double getAbsoluteYaw() {
      return yaw_absolute_angle;
    }

    double getIncrementalYaw() {
      return yaw_increment_angle;
    }

    void setInitialYawIncrement(double value) {
      initial_yaw_increment = value;
      set_increment = false;
    }

    double getRollRate_rad() {
      return gyroX;
    }

    double getPitchRate_rad() {
      return gyroY;
    }

    double getYawRate_rad() {
      return yaw_rate;
    }

    double getRollRate_deg() {
      return rad_to_deg(gyroX);
    }

    double getPitchRate_deg() {
      return rad_to_deg(gyroY);
    }

    double getYawRate_deg() {
      return rad_to_deg(yaw_rate);
    }



  private:
    const int MPU6050 = 0x68; // MPU6050 I2C address

    double accX, accY, accZ;
    double accX_offset, accY_offset, accZ_offset;
    double accVector[3][1];

    double gyroX, gyroY, gyroZ;
    double gyroX_offset, gyroY_offset, gyroZ_offset;

    const int QMC5883 = 0x0D; // QMC5883L I2C address

    double magX, magY, magZ;
    double b[3][1];
    double A_1[3][3];
    double magVector[3][1], magData[3][1];
    uint8_t DRDY;

    double North[3][1], Up[3][1], West[3][1], NorthCorrect[3][1];

    double roll, pitch, yaw;
    double Roll, Pitch, Yaw;
    double roll_rate, pitch_rate, yaw_rate;

    ////////////////////////////////////////////////////////
    bool set_increment = false, start_increment = false;
    double yaw_increment, new_yaw, old_yaw, initial_yaw_increment;

    double gain = 0.3, gain2 = 0.4, yaw_absolute_angle, yaw_increment_angle;

    double expFilter(double prev, double measured, double gain) {
      constrain(gain, 0, 1);
      return  prev + ( (1 - gain) * (measured - prev) );
    }




    void initIMU() {
      Wire.begin();                      // Initialize comunication

      Wire.beginTransmission(MPU6050);       // Start communication with MPU6050 // MPU=0x68
      Wire.write(0x6B);                  // Talk to the register 6B
      Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
      Wire.endTransmission();        //end the transmission

      // set up continous mode according to the QMC5883 datasheet
      Wire.beginTransmission(QMC5883);
      Wire.write(0x0B);
      Wire.write(0x01);
      Wire.endTransmission();

      Wire.beginTransmission(QMC5883);
      Wire.write(0x09);
      Wire.write(0x1D);
      Wire.endTransmission();
    }



    void getAccData() {
      // === Read acceleromter data === //
      Wire.beginTransmission(MPU6050);
      Wire.write(0x3B); // Start with register 0x3B (accEL_XOUT_H)
      Wire.endTransmission();
      Wire.requestFrom(MPU6050, 6); // Read 6 registers total, each axis value is stored in 2 registers

      //get raw accelerometer data and remove offset
      accX = (Wire.read() << 8 | Wire.read()) - accX_offset; // X-axis value
      accY = (Wire.read() << 8 | Wire.read()) - accY_offset; // Y-axis value
      accZ = (Wire.read() << 8 | Wire.read()) - accZ_offset; // Z-axis value


      //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
      accX = accX / 16384;
      accY = accY / 16384;
      accZ = accZ / 16384;

      //copy into acceleration vector variable
      accVector[0][0] = accX;
      accVector[1][0] = accY;
      accVector[2][0] = accZ;

      mat::DCM(accVector, accVector); //direction cosine matrices of the vector
    }



    void getMagData() {
      // === Read magnetometer data === //
      do {
        Wire.beginTransmission(QMC5883);
        Wire.write(0x06); // Start with register 0x00 (mag_XOUT_L)
        Wire.endTransmission();
        Wire.requestFrom(QMC5883, 1);
        DRDY = Wire.read();
      } while (!(DRDY & 0x01));

      Wire.beginTransmission(QMC5883);
      Wire.write(0x00); // Start with register 0x00 (mag_XOUT_L)
      Wire.endTransmission();
      Wire.requestFrom(QMC5883, 6);

      magX = (Wire.read() | Wire.read() << 8) * 0.0333; // X-axis value in uT
      magY = (Wire.read() | Wire.read() << 8) * 0.0333; // Y-axis value in uT
      magZ = (Wire.read() | Wire.read() << 8) * 0.0333; // Z-axis value in uT

      // copy into magData vector
      magData[0][0] = magX;
      magData[1][0] = magY;
      magData[2][0] = magZ;

      // magVector = A_1*(magData - b) using the A matrix and b vector to remove the magnetic offsets
      mat::sub(magData, magData, b);  //(magData - b) . store answer in magData
      mat::prod(magVector, A_1, magData); //matrix multiplication (A_1*magData). store answer in magVector
      mat::DCM(magVector, magVector); // direction cosine of magVector. store back in magVector
//      mat::printVect/or(magVector);
    }



    void getGyroData() {
      // === Read gyroscope data === //
      Wire.beginTransmission(MPU6050);
      Wire.write(0x43); // gyro data first register address 0x43
      Wire.endTransmission();
      Wire.requestFrom(MPU6050, 6); // Read 4 registers total, each axis value is stored in 2 registers

      // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
      gyroX = (Wire.read() << 8 | Wire.read()) - gyroX_offset;
      gyroY = (Wire.read() << 8 | Wire.read()) - gyroY_offset;
      gyroZ = (Wire.read() << 8 | Wire.read()) - gyroZ_offset;

      gyroX = gyroX / 131.0;
      gyroY = gyroY / 131.0;
      gyroZ = gyroZ / 131.0;
    }

    double deg_to_rad(double deg) {
      double rad = (deg * PI) / 180;
      return rad;
    }

    double rad_to_deg(double rad) {
      double deg = (rad * 180) / PI;
      return deg;
    }


    void ComputeRollPitchYaw() {
      getAccData();
      getMagData();
      getGyroData();

      // roll output is between -180 to 0 to +180
      roll = atan2( accVector[1][0], accVector[2][0] ) * 180 / PI; //acc_y/acc_z


      // pitch angle will be between -180 to 0 to +180
      pitch = atan2( (-1 * accVector[0][0]), sqrt((accVector[1][0] * accVector[1][0]) + (accVector[2][0] * accVector[2][0])) ) * 180 / PI; //-acc_x/(sqrt(acc_y^2 + acc_z^2))
      if ((accVector[0][0] < 0) && (accVector[2][0] < 0)) {
        pitch = 180 - pitch;
      }
      else if ((accVector[0][0] > 0) && (accVector[2][0] < 0)) {
        pitch = -180 - pitch;
      }


      mat::copy(Up, accVector);
      mat::cross(West, Up, magVector);
      mat::cross(North, West, Up);

      double cr = cos(radians(roll)), sr = sin(radians(roll));
      double cp = cos(radians(pitch)), sp = sin(radians(pitch));

      double Rroll[3][3] = {
        {1, 0, 0},
        {0, cr, (-1 * sr)},
        {0, sr, cr},
      };

      double Rpitch[3][3] = {
        {cp, 0, sp},
        {0, 1, 0},
        {(-1 * sp), 0, cp},
      };

      double vector[3][1];

      mat::prod(vector, Rroll, North);
      mat::prod(NorthCorrect, Rpitch, vector);

      // NorthCorrect[0][0] = (North[0][0]*cp) + (North[1][0]*sr*sp) + (North[2][0]*cr*sp);
      // NorthCorrect[1][0] =  (North[1][0]*cr) - (North[0][0]*sr);

      // pitch angle will be between -180 to 0 to +180
      yaw = atan2(-1 * NorthCorrect[1][0], NorthCorrect[0][0]) * 180 / PI;



      // this stores roll pitch and yaw absolute angles from 0 to 360
      if (roll < 0) Roll = roll + 360;
      else Roll = roll;

      if (pitch < 0) Pitch = pitch + 360;
      else Pitch = pitch;

      if (yaw < 0) Yaw = yaw + 360;
      else Yaw = yaw;

      compute_yaw_increment();

      yaw_absolute_angle = expFilter(yaw_absolute_angle, Yaw, gain);
      yaw_increment_angle = expFilter(yaw_increment_angle, yaw_increment, gain);

      yaw_rate = expFilter(yaw_rate, gyroZ, gain2);

    }


    void increment_angle() {
      new_yaw = Yaw;
      if ((new_yaw - old_yaw) > 180) {
        yaw_increment -= ((new_yaw - old_yaw) - 360);
      }
      else if ((new_yaw - old_yaw) < (-1 * 180)) {
        yaw_increment -= ((new_yaw - old_yaw) + 360);
      }
      else {
        yaw_increment += (new_yaw - old_yaw);
      }
      old_yaw = new_yaw;
    }


    void compute_yaw_increment() {
      if (!set_increment) {
        yaw_increment = initial_yaw_increment;
        set_increment = true;
      }

      if (!start_increment) {
        old_yaw = Yaw;
        start_increment = true;
      }

      increment_angle();
    }

};



Imu IMU; // creating the Imu object - datatype



/// IMU setup

double accOffset[3] = {-288.776, -249.408, -1959.296};
double gyroOffset[3] = {-163.0, -4.5, -72.5};


double A_matrix[3][3] = {
 { 0.02893807, -0.0008865,  0.0006017 },
 {-0.0008865,  0.03191024, -0.00035042},
 { 0.0006017, -0.00035042,  0.0303057 },
};

double b_vector[3][1] = {
 {-12.61894702},
 {-78.41404882},
 {-17.38327953},
};


void IMU_setup() {
  IMU.init();
  IMU.setAccOffset(accOffset[0], accOffset[1], accOffset[2]);
  IMU.setGyroOffset(gyroOffset[0], gyroOffset[1], gyroOffset[2]);
  IMU.setMagOffset(A_matrix, b_vector);
}
