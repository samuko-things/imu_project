
/*
 * Basic example code on how to control via I2C your geared DC motor with quadrature 
 * encoder which is already connected to the smc_driver shield module and have already
 * succesfully set up their velocity PID control using the smc_app GUI 
 * 
 * The code basically sends a low target velocity (in rad/s), waits for some time and then
 * sends a high target velocity (in rad/s). it also prints out the motors' angular positions
 * (in rad) and angular velocities (in rad/s).
 * 
 * you can copy the code and use it in your project as you will.
 */

// SMC i2c communication library
#include <samuko_mpu9250_i2c_comm.h>


int imuAddress = 4;
SamukoMPU9250I2cCommApi imu(imuAddress);

///////// my sepcial delay function ///////////////
void delayMs(int ms) {
  for (int i = 0; i < ms; i += 1) {
    delayMicroseconds(1000);
  }
}
//////////////////////////////////////////////////

float roll, pitch, yaw;

long prevSampleTime;
long sampleTime = 50; // millisec

void setup()
{
  // setup serial communication to print result on serial minitor               
  Serial.begin(115200);

  Serial.println("setting up I2C");
  // start i2c communication
  Wire.begin(); 
  Serial.println("imu i2c setup finished");

  for (int i = 0; i < 20; i += 1) {
    delay(1000);
    Serial.println(i);
  }

  prevSampleTime = millis();
}


 
void loop()
{  
  
  if ((millis() - prevSampleTime) >= sampleTime) {
    /* CODE SHOULD GO IN HERE*/

    imu.getRPY(roll, pitch, yaw);

    Serial.print(roll,4);
    Serial.print(", ");
    Serial.print(pitch,4);
    Serial.print(", ");
    Serial.println(yaw,4);

    Serial.println();
    
    prevSampleTime = millis();
  }


}
