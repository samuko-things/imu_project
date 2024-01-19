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
void updateAccOffset(){
  EEPROM.get(AX_OFFSET_ADDRESS, axOff);
  EEPROM.get(AY_OFFSET_ADDRESS, ayOff);
  EEPROM.get(AZ_OFFSET_ADDRESS, azOff);
}
void setAccOffset(float axOffset, float ayOffset, float azOffset){
  EEPROM.put(AX_OFFSET_ADDRESS, axOffset);
  EEPROM.put(AY_OFFSET_ADDRESS, ayOffset);
  EEPROM.put(AZ_OFFSET_ADDRESS, azOffset);
  updateAccOffset();
}

void updateGyroOffset(){
  EEPROM.get(GX_OFFSET_ADDRESS, gxOff);
  EEPROM.get(GY_OFFSET_ADDRESS, gyOff);
  EEPROM.get(GZ_OFFSET_ADDRESS, gzOff);
}
void setGyroOffset(float gxOffset, float gyOffset, float gzOffset){
  EEPROM.put(GX_OFFSET_ADDRESS, gxOffset);
  EEPROM.put(GY_OFFSET_ADDRESS, gyOffset);
  EEPROM.put(GZ_OFFSET_ADDRESS, gzOffset);
  updateGyroOffset();
}
//////////////////////////////////////////////////////////





//////////////////////////////////////////////////////////

void updateBvect(){
  EEPROM.get(B_VECT_X_ADDRESS, b_vect[0]);
  EEPROM.get(B_VECT_Y_ADDRESS, b_vect[1]);
  EEPROM.get(B_VECT_Z_ADDRESS, b_vect[2]);
}
void setBvect(float bvectX, float bvectY, float bvectZ){
  EEPROM.put(B_VECT_X_ADDRESS, bvectX);
  EEPROM.put(B_VECT_Y_ADDRESS, bvectY);
  EEPROM.put(B_VECT_Z_ADDRESS, bvectZ);
  updateBvect();
}


void updateAmatR0(){
  EEPROM.get(A_MAT_00_ADDRESS, A_mat[0][0]);
  EEPROM.get(A_MAT_01_ADDRESS, A_mat[0][1]);
  EEPROM.get(A_MAT_02_ADDRESS, A_mat[0][2]);
}
void setAmatR0(float a_mat00, float a_mat01, float a_mat02){
  EEPROM.put(A_MAT_00_ADDRESS, a_mat00);
  EEPROM.put(A_MAT_01_ADDRESS, a_mat01);
  EEPROM.put(A_MAT_02_ADDRESS, a_mat02);
  updateAmatR0();
}


void updateAmatR1(){
  EEPROM.get(A_MAT_10_ADDRESS, A_mat[1][0]);
  EEPROM.get(A_MAT_11_ADDRESS, A_mat[1][1]);
  EEPROM.get(A_MAT_12_ADDRESS, A_mat[1][2]);
}
void setAmatR1(float a_mat10, float a_mat11, float a_mat12){
  EEPROM.put(A_MAT_10_ADDRESS, a_mat10);
  EEPROM.put(A_MAT_11_ADDRESS, a_mat11);
  EEPROM.put(A_MAT_12_ADDRESS, a_mat12);
  updateAmatR1();
}


void updateAmatR2(){
  EEPROM.get(A_MAT_20_ADDRESS, A_mat[2][0]);
  EEPROM.get(A_MAT_21_ADDRESS, A_mat[2][1]);
  EEPROM.get(A_MAT_22_ADDRESS, A_mat[2][2]);
}
void setAmatR2(float a_mat20, float a_mat21, float a_mat22){
  EEPROM.put(A_MAT_20_ADDRESS, a_mat20);
  EEPROM.put(A_MAT_21_ADDRESS, a_mat21);
  EEPROM.put(A_MAT_22_ADDRESS, a_mat22);
  updateAmatR2();
}

//////////////////////////////////////////////////////////





//////////////////////////////////////////////////////////
void updateAngleVariance(){
  EEPROM.get(ROLL_ANGLE_VAR_ADDRESS, R_roll);
  EEPROM.get(PITCH_ANGLE_VAR_ADDRESS, R_pitch);
  EEPROM.get(YAW_ANGLE_VAR_ADDRESS, R_yaw);
}
void setAngleVariance(float rollVar, float pitchVar, float yawVar){
  EEPROM.put(ROLL_ANGLE_VAR_ADDRESS, rollVar);
  EEPROM.put(PITCH_ANGLE_VAR_ADDRESS, pitchVar);
  EEPROM.put(YAW_ANGLE_VAR_ADDRESS, yawVar);
  updateAngleVariance();
}


void updateRateVariance(){
  EEPROM.get(ROLL_RATE_VAR_ADDRESS, Q_roll);
  EEPROM.get(PITCH_RATE_VAR_ADDRESS, Q_pitch);
  EEPROM.get(YAW_RATE_VAR_ADDRESS, Q_yaw);
}
void setRateVariance(float rollVar, float pitchVar, float yawVar){
  EEPROM.put(ROLL_RATE_VAR_ADDRESS, rollVar);
  EEPROM.put(PITCH_RATE_VAR_ADDRESS, pitchVar);
  EEPROM.put(YAW_RATE_VAR_ADDRESS, yawVar);
  updateRateVariance();
}


void updateAccVariance(){
  EEPROM.get(ACC_X_VAR_ADDRESS, accx_variance);
  EEPROM.get(ACC_Y_VAR_ADDRESS, accy_variance);
  EEPROM.get(ACC_Z_VAR_ADDRESS, accz_variance);
}
void setAccVariance(float axVar, float ayVar, float azVar){
  EEPROM.put(ACC_X_VAR_ADDRESS, axVar);
  EEPROM.put(ACC_Y_VAR_ADDRESS, ayVar);
  EEPROM.put(ACC_Z_VAR_ADDRESS, azVar);
  updateAccVariance();
}
//////////////////////////////////////////////////////////////




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

  setAccOffset(0.0, 0.0, 0.0);
  setGyroOffset(0.0, 0.0, 0.0);
  setBvect(0.0, 0.0, 0.0);
  setAmatR0(0.0, 0.0, 0.0);
  setAmatR1(0.0, 0.0, 0.0);
  setAmatR2(0.0, 0.0, 0.0);
  setAngleVariance(0.0, 0.0, 0.0);
  setRateVariance(0.0, 0.0, 0.0);
  setAccVariance(0.0, 0.0, 0.0);
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

  updateAccOffset();
  updateGyroOffset();
  updateBvect();
  updateAmatR0();
  updateAmatR1();
  updateAmatR2();
  updateAngleVariance();
  updateRateVariance();
  updateAccVariance();
}
/////////////////////////////////////////////////////////////


#endif