#ifndef SERIAL_I2C_COMM_API_H
#define SERIAL_I2C_COMM_API_H
#include<Arduino.h>
#include "eeprom_setup.h"








void initLed(){
  pinMode(LED_BUILTIN, OUTPUT);
}

void onLed(){
  digitalWrite(LED_BUILTIN, HIGH);
}

void offLed(){
  digitalWrite(LED_BUILTIN, LOW);
}








///////// DIFFERENT TASK FOR SERIAL AND I2C COMMUNICATION //////////

/////////////////////////////////////////////////////////////////////////////////////
String sendRawAccData(){
  String data = String(axRaw, 4);
  data += ",";
  data += String(ayRaw, 4);
  data += ",";
  data += String(azRaw, 4);
  return data;
}

String sendCalAccData(){
  String data = String(axCal, 4);
  data += ",";
  data += String(ayCal, 4);
  data += ",";
  data += String(azCal, 4);
  return data;
}



String sendRawGyroData(){
  String data = String(gxRaw, 4);
  data += ",";
  data += String(gyRaw, 4);
  data += ",";
  data += String(gzRaw, 4);
  return data;
}

String sendCalGyroData(){
  String data = String(gxCal, 4);
  data += ",";
  data += String(gyCal, 4);
  data += ",";
  data += String(gzCal, 4);
  return data;
}




String sendRawMagData(){
  String data = String(mxRaw, 4);
  data += ",";
  data += String(myRaw, 4);
  data += ",";
  data += String(mzRaw, 4);
  return data;
}

String sendCalMagData(){
  String data = String(mxCal, 4);
  data += ",";
  data += String(myCal, 4);
  data += ",";
  data += String(mzCal, 4);
  return data;
}




String sendRPY_rad(){
  String data = String(roll, 4);
  data += ",";
  data += String(pitch, 4);
  data += ",";
  data += String(yaw, 4);
  return data;
}

String sendRPY_rate(){
  String data = String(roll_rate, 4);
  data += ",";
  data += String(pitch_rate, 4);
  data += ",";
  data += String(yaw_rate, 4);
  return data;
}

///////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////
String resetEEPROM(){
  setFIRST_TIME(0);
  return "1";
}
////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////
String updateAxOffset(float axOffset){
  setAxOffset(axOffset);
  axOff = getAxOffset();
  return "1";
}
String sendAxOffset(){
  return String(axOff,10);
}



String updateAyOffset(float ayOffset){
  setAyOffset(ayOffset);
  ayOff = getAyOffset();
  return "1";
}
String sendAyOffset(){
  return String(ayOff,10);
}



String updateAzOffset(float azOffset){
  setAzOffset(azOffset);
  azOff = getAzOffset();
  return "1";
}
String sendAzOffset(){
  return String(azOff,10);
}
////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////
String updateGxOffset(float gxOffset){
  setGxOffset(gxOffset);
  gxOff = getGxOffset();
  return "1";
}
String sendGxOffset(){
  return String(gxOff,10);
}



String updateGyOffset(float gyOffset){
  setGyOffset(gyOffset);
  gyOff = getGyOffset();
  return "1";
}
String sendGyOffset(){
  return String(gyOff,10);
}



String updateGzOffset(float gzOffset){
  setGzOffset(gzOffset);
  gzOff = getGzOffset();
  return "1";
}
String sendGzOffset(){
  return String(gzOff,10);
}
////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////
String updateBvectX(float bvectX){
  setBvectX(bvectX);
  b_vect[0] = getBvectX();
  return "1";
}
String sendBvectX(){
  return String(b_vect[0],10);
}


String updateBvectY(float bvectY){
  setBvectY(bvectY);
  b_vect[1] = getBvectY();
  return "1";
}
String sendBvectY(){
  return String(b_vect[1],10);
}


String updateBvectZ(float bvectZ){
  setBvectZ(bvectZ);
  b_vect[2] = getBvectZ();
  return "1";
}
String sendBvectZ(){
  return String(b_vect[2],10);
}
////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////
String updateAmat00(float amat00){
  setAmat00(amat00);
  A_mat[0][0] = getAmat00();
  return "1";
}
String sendAmat00(){
  return String(A_mat[0][0],10);
}


String updateAmat01(float amat01){
  setAmat01(amat01);
  A_mat[0][1] = getAmat01();
  return "1";
}
String sendAmat01(){
  return String(A_mat[0][1],10);
}


String updateAmat02(float amat02){
  setAmat02(amat02);
  A_mat[0][2] = getAmat02();
  return "1";
}
String sendAmat02(){
  return String(A_mat[0][2],10);
}


String updateAmat10(float amat10){
  setAmat10(amat10);
  A_mat[1][0] = getAmat10();
  return "1";
}
String sendAmat10(){
  return String(A_mat[1][0],10);
}


String updateAmat11(float amat11){
  setAmat11(amat11);
  A_mat[1][1] = getAmat11();
  return "1";
}
String sendAmat11(){
  return String(A_mat[1][1],10);
}


String updateAmat12(float amat12){
  setAmat12(amat12);
  A_mat[1][2] = getAmat12();
  return "1";
}
String sendAmat12(){
  return String(A_mat[1][2],10);
}


String updateAmat20(float amat20){
  setAmat20(amat20);
  A_mat[2][0] = getAmat20();
  return "1";
}
String sendAmat20(){
  return String(A_mat[2][0],10);
}


String updateAmat21(float amat21){
  setAmat21(amat21);
  A_mat[2][1] = getAmat21();
  return "1";
}
String sendAmat21(){
  return String(A_mat[2][1],10);
}


String updateAmat22(float amat22){
  setAmat22(amat22);
  A_mat[2][2] = getAmat22();
  return "1";
}
String sendAmat22(){
  return String(A_mat[2][2],10);
}
///////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////
String updateRollAngleVariance(float rollAngleVariance){
  setRollAngleVariance(rollAngleVariance);
  R_roll = getRollAngleVariance();
  return "1";
}
String sendRollAngleVariance(){
  return String(R_roll,10);
}


String updatePitchAngleVariance(float pitchAngleVariance){
  setPitchAngleVariance(pitchAngleVariance);
  R_pitch = getPitchAngleVariance();
  return "1";
}
String sendPitchAngleVariance(){
  return String(R_pitch,10);
}


String updateYawAngleVariance(float yawAngleVariance){
  setYawAngleVariance(yawAngleVariance);
  R_yaw = getYawAngleVariance();
  return "1";
}
String sendYawAngleVariance(){
  return String(R_yaw,10);
}
//////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////
String updateRollRateVariance(float rollRateVariance){
  setRollRateVariance(rollRateVariance);
  Q_roll = getRollRateVariance();
  return "1";
}
String sendRollRateVariance(){
  return String(Q_roll,10);
}


String updatePitchRateVariance(float pitchRateVariance){
  setPitchRateVariance(pitchRateVariance);
  Q_pitch = getPitchRateVariance();
  return "1";
}
String sendPitchRateVariance(){
  return String(Q_pitch,10);
}


String updateYawRateVariance(float yawRateVariance){
  setYawRateVariance(yawRateVariance);
  Q_yaw = getYawRateVariance();
  return "1";
}
String sendYawRateVariance(){
  return String(Q_yaw,10);
}
//////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////
String updateAccxVariance(float accxVariance){
  setAccxVariance(accxVariance);
  accx_variance = getAccxVariance();
  return "1";
}
String sendAccxVariance(){
  return String(accx_variance,10);
}

String updateAccyVariance(float accyVariance){
  setAccyVariance(accyVariance);
  accy_variance = getAccyVariance();
  return "1";
}
String sendAccyVariance(){
  return String(accy_variance,10);
}

String updateAcczVariance(float acczVariance){
  setAcczVariance(acczVariance);
  accz_variance = getAcczVariance();
  return "1";
}
String sendAcczVariance(){
  return String(accz_variance,10);
}
////////////////////////////////////////////////////////////////////






























///////////////// SERIAL COMMUNICATION //////////////////////
String ser_msg = "";

String serMsg = "", serMsgBuffer, serDataBuffer[3];

void serialReceiveAndSendData() {
  int indexPos = 0, i = 0;

  if (Serial.available() > 0) {
    while (Serial.available())
    {
      serMsg = Serial.readString();
    }
    serMsg.trim();
    if (serMsg != "") {
      do {
        indexPos = serMsg.indexOf(',');
        if (indexPos != -1) {
          serMsgBuffer = serMsg.substring(0, indexPos);
          serMsg = serMsg.substring(indexPos + 1, serMsg.length());
          serDataBuffer[i] = serMsgBuffer;
          serMsgBuffer = "";
        }
        else {
          if (serMsg.length() > 0)
            serDataBuffer[i] = serMsg;
        }
        i += 1;
      } while (indexPos >= 0);
    }


    if (serDataBuffer[0] != ""){

      /////////////// FUNCTION CALLS /////////////////////

      if(serDataBuffer[0] == "acc-cal"){
        ser_msg = sendCalAccData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "acc-raw"){
        ser_msg = sendRawAccData();
        Serial.println(ser_msg);
        ser_msg = "";
      }
      
      else if(serDataBuffer[0] == "gyro-cal"){
        ser_msg = sendCalGyroData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "gyro-raw"){
        ser_msg = sendRawGyroData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "mag-cal"){
        ser_msg = sendCalMagData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "mag-raw"){
        ser_msg = sendRawMagData();
        Serial.println(ser_msg);
        ser_msg = "";
      }


      else if(serDataBuffer[0] == "rpy-rad"){
        ser_msg = sendRPY_rad();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "rpy-rate"){
        ser_msg = sendRPY_rate();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "ax-off") {
        if (serDataBuffer[1]=="") ser_msg = sendAxOffset();
        else ser_msg = updateAxOffset(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "ay-off") {
        if (serDataBuffer[1]=="") ser_msg = sendAyOffset();
        else ser_msg = updateAyOffset(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "az-off") {
        if (serDataBuffer[1]=="") ser_msg = sendAzOffset();
        else ser_msg = updateAzOffset(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "gx-off") {
        if (serDataBuffer[1]=="") ser_msg = sendGxOffset();
        else ser_msg = updateGxOffset(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "gy-off") {
        if (serDataBuffer[1]=="") ser_msg = sendGyOffset();
        else ser_msg = updateGyOffset(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "gz-off") {
        if (serDataBuffer[1]=="") ser_msg = sendGzOffset();
        else ser_msg = updateGzOffset(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "bvect-x") {
        if (serDataBuffer[1]=="") ser_msg = sendBvectX();
        else ser_msg = updateBvectX(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "bvect-y") {
        if (serDataBuffer[1]=="") ser_msg = sendBvectY();
        else ser_msg = updateBvectY(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "bvect-z") {
        if (serDataBuffer[1]=="") ser_msg = sendBvectZ();
        else ser_msg = updateBvectZ(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "amat-00") {
        if (serDataBuffer[1]=="") ser_msg = sendAmat00();
        else ser_msg = updateAmat00(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "amat-01") {
        if (serDataBuffer[1]=="") ser_msg = sendAmat01();
        else ser_msg = updateAmat01(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "amat-02") {
        if (serDataBuffer[1]=="") ser_msg = sendAmat02();
        else ser_msg = updateAmat02(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "amat-10") {
        if (serDataBuffer[1]=="") ser_msg = sendAmat10();
        else ser_msg = updateAmat10(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "amat-11") {
        if (serDataBuffer[1]=="") ser_msg = sendAmat11();
        else ser_msg = updateAmat11(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "amat-12") {
        if (serDataBuffer[1]=="") ser_msg = sendAmat12();
        else ser_msg = updateAmat12(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "amat-20") {
        if (serDataBuffer[1]=="") ser_msg = sendAmat20();
        else ser_msg = updateAmat20(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "amat-21") {
        if (serDataBuffer[1]=="") ser_msg = sendAmat21();
        else ser_msg = updateAmat21(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "amat-22") {
        if (serDataBuffer[1]=="") ser_msg = sendAmat22();
        else ser_msg = updateAmat22(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "rAng-var") {
        if (serDataBuffer[1]=="") ser_msg = sendRollAngleVariance();
        else ser_msg = updateRollAngleVariance(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "rRate-var") {
        if (serDataBuffer[1]=="") ser_msg = sendRollRateVariance();
        else ser_msg = updateRollRateVariance(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "pAng-var") {
        if (serDataBuffer[1]=="") ser_msg = sendPitchAngleVariance();
        else ser_msg = updatePitchAngleVariance(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "pRate-var") {
        if (serDataBuffer[1]=="") ser_msg = sendPitchRateVariance();
        else ser_msg = updatePitchRateVariance(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "yAng-var") {
        if (serDataBuffer[1]=="") ser_msg = sendYawAngleVariance();
        else ser_msg = updateYawAngleVariance(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "yRate-var") {
        if (serDataBuffer[1]=="") ser_msg = sendYawRateVariance();
        else ser_msg = updateYawRateVariance(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "accx-var") {
        if (serDataBuffer[1]=="") ser_msg = sendAccxVariance();
        else ser_msg = updateAccxVariance(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "accy-var") {
        if (serDataBuffer[1]=="") ser_msg = sendAccyVariance();
        else ser_msg = updateAccyVariance(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "accz-var") {
        if (serDataBuffer[1]=="") ser_msg = sendAcczVariance();
        else ser_msg = updateAcczVariance(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "reset") {
        ser_msg = resetEEPROM();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      ////////////////////////////////////////////////////////////////////////////////////

    } else {
      ser_msg = "0";
      Serial.println(ser_msg);
      ser_msg = "";
    }
  }
  
  serMsg = "";
  serMsgBuffer = "";
  serDataBuffer[0] = "";
  serDataBuffer[1] = "";
  serDataBuffer[2] = "";
}
//////////////////////////////////////////////////////////////////////////





#endif