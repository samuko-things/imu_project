#ifndef EEPROM_SETUP_H
#define EEPROM_SETUP_H



/////////////////// STORING AND READING PARAMETERS FROM EEPROM /////////////////
#include <EEPROM.h>
#include "global_eeprom_variables.h"

int FIRST_TIME_ADDRESS = 0;

int AX_OFFSET_ADDRESS = 4;
int AY_OFFSET_ADDRESS = 8;
int AZ_OFFSET_ADDRESS = 12;

int GX_OFFSET_ADDRESS = 16;
int GY_OFFSET_ADDRESS = 20;
int GZ_OFFSET_ADDRESS = 24;

int B_VECT_X_ADDRESS = 28;
int B_VECT_Y_ADDRESS = 32;
int B_VECT_Z_ADDRESS = 36;

int A_MAT_00_ADDRESS = 40;
int A_MAT_01_ADDRESS = 44;
int A_MAT_02_ADDRESS = 48;

int A_MAT_10_ADDRESS = 52;
int A_MAT_11_ADDRESS = 56;
int A_MAT_12_ADDRESS = 60;

int A_MAT_20_ADDRESS = 64;
int A_MAT_21_ADDRESS = 68;
int A_MAT_22_ADDRESS = 72;

int ROLL_ANGLE_VAR_ADDRESS = 76;
int PITCH_ANGLE_VAR_ADDRESS = 80;
int YAW_ANGLE_VAR_ADDRESS = 84;

int ROLL_RATE_VAR_ADDRESS = 88;
int PITCH_RATE_VAR_ADDRESS = 92;
int YAW_RATE_VAR_ADDRESS = 96;

int ACC_X_VAR_ADDRESS = 100;
int ACC_Y_VAR_ADDRESS = 104;
int ACC_Z_VAR_ADDRESS = 108;







//////////////////////////////////////////////////////////

float getAxOffset(){
  float axOffset;
  EEPROM.get(AX_OFFSET_ADDRESS, axOffset);
  return axOffset;
}
void setAxOffset(float axOffset){
  EEPROM.put(AX_OFFSET_ADDRESS, axOffset);
  axOff = getAxOffset();
}


float getAyOffset(){
  float ayOffset;
  EEPROM.get(AY_OFFSET_ADDRESS, ayOffset);
  return ayOffset;
}
void setAyOffset(float ayOffset){
  EEPROM.put(AY_OFFSET_ADDRESS, ayOffset);
  ayOff = getAyOffset();
}


float getAzOffset(){
  float azOffset;
  EEPROM.get(AZ_OFFSET_ADDRESS, azOffset);
  return azOffset;
}
void setAzOffset(float azOffset){
  EEPROM.put(AZ_OFFSET_ADDRESS, azOffset);
  azOff = getAzOffset();
}

//////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////

float getGxOffset(){
  float gxOffset;
  EEPROM.get(GX_OFFSET_ADDRESS, gxOffset);
  return gxOffset;
}
void setGxOffset(float gxOffset){
  EEPROM.put(GX_OFFSET_ADDRESS, gxOffset);
  gxOff = getGxOffset();
}


float getGyOffset(){
  float gyOffset;
  EEPROM.get(GY_OFFSET_ADDRESS, gyOffset);
  return gyOffset;
}
void setGyOffset(float gyOffset){
  EEPROM.put(GY_OFFSET_ADDRESS, gyOffset);
  gyOff = getGyOffset();
}


float getGzOffset(){
  float gzOffset;
  EEPROM.get(GZ_OFFSET_ADDRESS, gzOffset);
  return gzOffset;
}
void setGzOffset(float gzOffset){
  EEPROM.put(GZ_OFFSET_ADDRESS, gzOffset);
  gzOff = getGzOffset();
}

//////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////

float getBvectX(){
  float bvectX;
  EEPROM.get(B_VECT_X_ADDRESS, bvectX);
  return bvectX;
}
void setBvectX(float bvectX){
  EEPROM.put(B_VECT_X_ADDRESS, bvectX);
  b_vect[0] = getBvectX();
}


float getBvectY(){
  float bvectY;
  EEPROM.get(B_VECT_Y_ADDRESS, bvectY);
  return bvectY;
}
void setBvectY(float bvectY){
  EEPROM.put(B_VECT_Y_ADDRESS, bvectY);
  b_vect[1] = getBvectY();
}


float getBvectZ(){
  float bvectZ;
  EEPROM.get(B_VECT_Z_ADDRESS, bvectZ);
  return bvectZ;
}
void setBvectZ(float bvectZ){
  EEPROM.put(B_VECT_Z_ADDRESS, bvectZ);
  b_vect[2] = getBvectZ();
}

//////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////

float getAmat00(){
  float amat00;
  EEPROM.get(A_MAT_00_ADDRESS, amat00);
  return amat00;
}
void setAmat00(float amat00){
  EEPROM.put(A_MAT_00_ADDRESS, amat00);
  A_mat[0][0] = getAmat00();
}


float getAmat01(){
  float amat01;
  EEPROM.get(A_MAT_01_ADDRESS, amat01);
  return amat01;
}
void setAmat01(float amat01){
  EEPROM.put(A_MAT_01_ADDRESS, amat01);
  A_mat[0][1] = getAmat01();
}


float getAmat02(){
  float amat02;
  EEPROM.get(A_MAT_02_ADDRESS, amat02);
  return amat02;
}
void setAmat02(float amat02){
  EEPROM.put(A_MAT_02_ADDRESS, amat02);
  A_mat[0][2] = getAmat02();
}


float getAmat10(){
  float amat10;
  EEPROM.get(A_MAT_10_ADDRESS, amat10);
  return amat10;
}
void setAmat10(float amat10){
  EEPROM.put(A_MAT_10_ADDRESS, amat10);
  A_mat[1][0] = getAmat10();
}


float getAmat11(){
  float amat11;
  EEPROM.get(A_MAT_11_ADDRESS, amat11);
  return amat11;
}
void setAmat11(float amat11){
  EEPROM.put(A_MAT_11_ADDRESS, amat11);
  A_mat[1][1] = getAmat11();
}


float getAmat12(){
  float amat12;
  EEPROM.get(A_MAT_12_ADDRESS, amat12);
  return amat12;
}
void setAmat12(float amat12){
  EEPROM.put(A_MAT_12_ADDRESS, amat12);
  A_mat[1][2] = getAmat12();
}



float getAmat20(){
  float amat20;
  EEPROM.get(A_MAT_20_ADDRESS, amat20);
  return amat20;
}
void setAmat20(float amat20){
  EEPROM.put(A_MAT_20_ADDRESS, amat20);
  A_mat[2][0] = getAmat20();
}


float getAmat21(){
  float amat21;
  EEPROM.get(A_MAT_21_ADDRESS, amat21);
  return amat21;
}
void setAmat21(float amat21){
  EEPROM.put(A_MAT_21_ADDRESS, amat21);
  A_mat[2][1] = getAmat21();
}


float getAmat22(){
  float amat22;
  EEPROM.get(A_MAT_22_ADDRESS, amat22);
  return amat22;
}
void setAmat22(float amat22){
  EEPROM.put(A_MAT_22_ADDRESS, amat22);
  A_mat[2][2] = getAmat22();
}

//////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////

float getRollAngleVariance(){
  float rollAngleVariance;
  EEPROM.get(ROLL_ANGLE_VAR_ADDRESS, rollAngleVariance);
  return rollAngleVariance;
}
void setRollAngleVariance(float rollAngleVariance){
  EEPROM.put(ROLL_ANGLE_VAR_ADDRESS, rollAngleVariance);
  R_roll = getRollAngleVariance();
}


float getPitchAngleVariance(){
  float pitchAngleVariance;
  EEPROM.get(PITCH_ANGLE_VAR_ADDRESS, pitchAngleVariance);
  return pitchAngleVariance;
}
void setPitchAngleVariance(float pitchAngleVariance){
  EEPROM.put(PITCH_ANGLE_VAR_ADDRESS, pitchAngleVariance);
  R_pitch = getPitchAngleVariance();
}



float getYawAngleVariance(){
  float yawAngleVariance;
  EEPROM.get(YAW_ANGLE_VAR_ADDRESS, yawAngleVariance);
  return yawAngleVariance;
}
void setYawAngleVariance(float yawAngleVariance){
  EEPROM.put(YAW_ANGLE_VAR_ADDRESS, yawAngleVariance);
  R_yaw = getYawAngleVariance();
}

//////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////

float getRollRateVariance(){
  float rollRateVariance;
  EEPROM.get(ROLL_RATE_VAR_ADDRESS, rollRateVariance);
  return rollRateVariance;
}
void setRollRateVariance(float rollRateVariance){
  EEPROM.put(ROLL_RATE_VAR_ADDRESS, rollRateVariance);
  Q_roll = getRollRateVariance();
}


float getPitchRateVariance(){
  float pitchRateVariance;
  EEPROM.get(PITCH_RATE_VAR_ADDRESS, pitchRateVariance);
  return pitchRateVariance;
}
void setPitchRateVariance(float pitchRateVariance){
  EEPROM.put(PITCH_RATE_VAR_ADDRESS, pitchRateVariance);
  Q_pitch = getPitchRateVariance();
}



float getYawRateVariance(){
  float yawRateVariance;
  EEPROM.get(YAW_RATE_VAR_ADDRESS, yawRateVariance);
  return yawRateVariance;
}
void setYawRateVariance(float yawRateVariance){
  EEPROM.put(YAW_RATE_VAR_ADDRESS, yawRateVariance);
  Q_yaw = getYawRateVariance();
}

//////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////
float getAccxVariance(){
  float accxVariance;
  EEPROM.get(ACC_X_VAR_ADDRESS, accxVariance);
  return accxVariance;
}
void setAccxVariance(float accxVariance){
  EEPROM.put(ACC_X_VAR_ADDRESS, accxVariance);
  accx_variance = getAccxVariance();
}

float getAccyVariance(){
  float accyVariance;
  EEPROM.get(ACC_Y_VAR_ADDRESS, accyVariance);
  return accyVariance;
}
void setAccyVariance(float accyVariance){
  EEPROM.put(ACC_Y_VAR_ADDRESS, accyVariance);
  accy_variance = getAccyVariance();
}

float getAcczVariance(){
  float acczVariance;
  EEPROM.get(ACC_Z_VAR_ADDRESS, acczVariance);
  return acczVariance;
}
void setAcczVariance(float acczVariance){
  EEPROM.put(ACC_Z_VAR_ADDRESS, acczVariance);
  accz_variance = getAcczVariance();
}
//////////////////////////////////////////////////////////////






/////////////////////////////////////////
// int getI2CADDRESS(){
//   float address;
//   EEPROM.get(I2C_ADDRESS, address);
//   return (int)address;
// }
// void setI2CADDRESS(int address){
//   EEPROM.put(I2C_ADDRESS, (float)address);
//   i2cAddress = getI2CADDRESS();
//   Wire.begin(i2cAddress);
// }
///////////////////////////////////////////



////////////////////////////////////////////////
void setFIRST_TIME(int val){
  EEPROM.put(FIRST_TIME_ADDRESS, (float)val);
}
int getFIRST_TIME(){
  float firstTime;
  EEPROM.get(FIRST_TIME_ADDRESS, firstTime);
  return (int)firstTime;
}
/////////////////////////////////////////////////
















///////////////////////////////////////////////////////////

void resetAllParams(){

  setAxOffset(0.00);
  setAyOffset(0.00);
  setAzOffset(0.00);

  setGxOffset(0.00);
  setGyOffset(0.00);
  setGzOffset(0.00);

  setBvectX(0.00);
  setBvectY(0.00);
  setBvectZ(0.00);

  setAmat00(0.00);
  setAmat01(0.00);
  setAmat02(0.00);

  setAmat10(0.00);
  setAmat11(0.00);
  setAmat12(0.00);

  setAmat20(0.00);
  setAmat21(0.00);
  setAmat22(0.00);

  setRollAngleVariance(0.00);
  setPitchAngleVariance(0.00);
  setYawAngleVariance(0.00);

  setRollRateVariance(0.00);
  setPitchRateVariance(0.00);
  setYawRateVariance(0.00);

  setAccxVariance(0.00);
  setAccyVariance(0.00);
  setAcczVariance(0.00);

}



void initEEPROMparamsStorage(){
  int isFirstTime, setupCode = 11111; // please do not change
  isFirstTime = getFIRST_TIME();
  if(isFirstTime != setupCode){ //if not equal to 11111
    setFIRST_TIME(setupCode);
    resetAllParams();
    // Serial.println("reset all eeprom paramas");
  }
}



void updateGlobalParamsFromEERPOM(){
  initEEPROMparamsStorage();

  axOff = getAxOffset();
  ayOff = getAyOffset();
  azOff = getAzOffset();

  gxOff = getGxOffset();
  gyOff = getGyOffset();
  gzOff = getGzOffset();

  b_vect[0] = getBvectX();
  b_vect[1] = getBvectY();
  b_vect[2] = getBvectZ();

  A_mat[0][0] = getAmat00();
  A_mat[0][1] = getAmat01();
  A_mat[0][2] = getAmat02();

  A_mat[1][0] = getAmat10();
  A_mat[1][1] = getAmat11();
  A_mat[1][2] = getAmat12();

  A_mat[2][0] = getAmat20();
  A_mat[2][1] = getAmat21();
  A_mat[2][2] = getAmat22();

  R_roll = getRollAngleVariance();
  R_pitch = getPitchAngleVariance();
  R_yaw = getYawAngleVariance();

  Q_roll = getRollRateVariance();
  Q_pitch = getPitchRateVariance();
  Q_yaw = getYawRateVariance();

  accx_variance = getAccxVariance();
  accy_variance = getAccyVariance();
  accz_variance = getAcczVariance();
}
/////////////////////////////////////////////////////////////


#endif