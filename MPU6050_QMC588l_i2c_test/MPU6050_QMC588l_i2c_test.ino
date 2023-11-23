#include "IMU.h"

long old_time1, new_time1;
long setup_delay_time = 3000;

void setup() {
  Serial.begin(9600);
  
  IMU_setup();
  old_time1 = millis();
  new_time1 = millis();
  IMU.setInitialYawIncrement(0);
  while ((new_time1 - old_time1) < setup_delay_time) {
    IMU.computeRollPitchYaw();
    new_time1 = millis();

    Serial.println(new_time1 - old_time1);
  }
  Serial.println("finished IMU setup");
  
  IMU.setInitialYawIncrement(0);

}

void loop() {
  IMU.computeRollPitchYaw();
  double inc_angle = IMU.getIncrementalYaw();
  double abs_angle = IMU.getAbsoluteYaw();
  Serial.print(int(inc_angle));
  Serial.print(" , ");
  Serial.println(int(abs_angle));

  delay(10);
}
