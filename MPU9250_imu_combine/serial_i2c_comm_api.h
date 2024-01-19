#ifndef SERIAL_I2C_COMM_API_H
#define SERIAL_I2C_COMM_API_H
#include <Arduino.h>
#include <Wire.h>
#include "eeprom_setup.h"

void initLed()
{
  pinMode(LED_BUILTIN, OUTPUT);
}

void onLed()
{
  digitalWrite(LED_BUILTIN, HIGH);
}

void offLed()
{
  digitalWrite(LED_BUILTIN, LOW);
}




///////// DIFFERENT TASK FOR SERIAL AND I2C COMMUNICATION //////////

//////////////////////////////////////////////////////////////////////
String resetEEPROM(){
  setFIRST_TIME(0);
  return "1";
}
////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////

String sendRawAccData(){
  String data = String(axRaw, 6);
  data += ",";
  data += String(ayRaw, 6);
  data += ",";
  data += String(azRaw, 6);
  return data;
}

String sendCalAccData(){
  String data = String(axCal, 6);
  data += ",";
  data += String(ayCal, 6);
  data += ",";
  data += String(azCal, 6);
  return data;
}


String sendOffAccData(){
  String data = String(axOff, 6);
  data += ",";
  data += String(ayOff, 6);
  data += ",";
  data += String(azOff, 6);
  return data;
}


String sendRawGyroData(){
  String data = String(gxRaw, 6);
  data += ",";
  data += String(gyRaw, 6);
  data += ",";
  data += String(gzRaw, 6);
  return data;
}

String sendCalGyroData(){
  String data = String(gxCal, 6);
  data += ",";
  data += String(gyCal, 6);
  data += ",";
  data += String(gzCal, 6);
  return data;
}

String sendOffGyroData(){
  String data = String(gxOff, 6);
  data += ",";
  data += String(gyOff, 6);
  data += ",";
  data += String(gzOff, 6);
  return data;
}


String sendRawMagData(){
  String data = String(mxRaw, 6);
  data += ",";
  data += String(myRaw, 6);
  data += ",";
  data += String(mzRaw, 6);
  return data;
}

String sendCalMagData(){
  String data = String(mxCal, 6);
  data += ",";
  data += String(myCal, 6);
  data += ",";
  data += String(mzCal, 6);
  return data;
}

String sendBvectMagData(){
  String data = String(b_vect[0], 6);
  data += ",";
  data += String(b_vect[1], 6);
  data += ",";
  data += String(b_vect[2], 6);
  return data;
}

String sendAmatR0MagData(){
  String data = String(A_mat[0][0], 6);
  data += ",";
  data += String(A_mat[0][1], 6);
  data += ",";
  data += String(A_mat[0][2], 6);
  return data;
}

String sendAmatR1MagData(){
  String data = String(A_mat[1][0], 6);
  data += ",";
  data += String(A_mat[1][1], 6);
  data += ",";
  data += String(A_mat[1][2], 6);
  return data;
}

String sendAmatR2MagData(){
  String data = String(A_mat[2][0], 6);
  data += ",";
  data += String(A_mat[2][1], 6);
  data += ",";
  data += String(A_mat[2][2], 6);
  return data;
}

String sendAngleVariance(){
  String data = String(R_roll, 6);
  data += ",";
  data += String(R_pitch, 6);
  data += ",";
  data += String(R_yaw, 6);
  return data;
}

String sendRateVariance(){
  String data = String(Q_roll, 6);
  data += ",";
  data += String(Q_pitch, 6);
  data += ",";
  data += String(Q_yaw, 6);
  return data;
}

String sendAccVariance(){
  String data = String(accx_variance, 6);
  data += ",";
  data += String(accy_variance, 6);
  data += ",";
  data += String(accz_variance, 6);
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

String sendRPY_est()
{
  String data = String(roll_est, 6);
  data += ",";
  data += String(pitch_est, 6);
  data += ",";
  data += String(yaw_est, 6);
  return data;
}

String send_heading()
{
  String data = String(heading_est, 6);
  return data;
}
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
String updateAccelerationOffset(float axOffset, float ayOffset, float azOffset){
  setAccOffset(axOffset, ayOffset, azOffset);
  return "1";
}

String updateGyroscopeOffset(float gxOffset, float gyOffset, float gzOffset){
  setGyroOffset(gxOffset, gyOffset, gzOffset);
  return "1";
}

String updateBvectMagOffset(float val1, float val2, float val3){
  setBvect(val1, val2, val3);
  return "1";
}

String updateAmatR0MagOffset(float val1, float val2, float val3){
  setAmatR0(val1, val2, val3);
  return "1";
}

String updateAmatR1MagOffset(float val1, float val2, float val3){
  setAmatR1(val1, val2, val3);
  return "1";
}

String updateAmatR2MagOffset(float val1, float val2, float val3){
  setAmatR2(val1, val2, val3);
  return "1";
}

String updateLinAccVariance(float val1, float val2, float val3){
  setAccVariance(val1, val2, val3);
  return "1";
}

String updateRPYAngleVariance(float val1, float val2, float val3){
  setAngleVariance(val1, val2, val3);
  return "1";
}

String updateRPYRateVariance(float val1, float val2, float val3){
  setRateVariance(val1, val2, val3);
  return "1";
}
//////////////////////////////////////////////////////////////////










///////////////// SERIAL COMMUNICATION //////////////////////
String ser_msg = "";

String serMsg = "", serMsgBuffer, serDataBuffer[4];

void serialReceiveAndSendData()
{
  int indexPos = 0, i = 0;

  if (Serial.available() > 0)
  {
    while (Serial.available())
    {
      serMsg = Serial.readString();
    }
    serMsg.trim();
    if (serMsg != "")
    {
      do
      {
        indexPos = serMsg.indexOf(',');
        if (indexPos != -1)
        {
          serMsgBuffer = serMsg.substring(0, indexPos);
          serMsg = serMsg.substring(indexPos + 1, serMsg.length());
          serDataBuffer[i] = serMsgBuffer;
          serMsgBuffer = "";
        }
        else
        {
          if (serMsg.length() > 0)
            serDataBuffer[i] = serMsg;
        }
        i += 1;
      } while (indexPos >= 0);
    }

    if (serDataBuffer[0] != "")
    {

      /////////////// FUNCTION CALLS /////////////////////

      if (serDataBuffer[0] == "acc-raw")
      {
        ser_msg = sendRawAccData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "acc-cal")
      {
        ser_msg = sendCalAccData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "acc-off") {
        if (serDataBuffer[1]=="") ser_msg = sendOffAccData();
        else ser_msg = updateAccelerationOffset(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "gyro-raw")
      {
        ser_msg = sendRawGyroData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "gyro-cal")
      {
        ser_msg = sendCalGyroData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "gyro-off") {
        if (serDataBuffer[1]=="") ser_msg = sendOffGyroData();
        else ser_msg = updateGyroscopeOffset(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "mag-raw")
      {
        ser_msg = sendRawMagData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "mag-cal")
      {
        ser_msg = sendCalMagData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "mag-bvect") {
        if (serDataBuffer[1]=="") ser_msg = sendBvectMagData();
        else ser_msg = updateBvectMagOffset(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "mag-amat0") {
        if (serDataBuffer[1]=="") ser_msg = sendAmatR0MagData();
        else ser_msg = updateAmatR0MagOffset(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "mag-amat1") {
        if (serDataBuffer[1]=="") ser_msg = sendAmatR1MagData();
        else ser_msg = updateAmatR1MagOffset(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "mag-amat2") {
        if (serDataBuffer[1]=="") ser_msg = sendAmatR2MagData();
        else ser_msg = updateAmatR2MagOffset(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "rpy-ang-var") {
        if (serDataBuffer[1]=="") ser_msg = sendAngleVariance();
        else ser_msg = updateRPYAngleVariance(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "rpy-rate-var") {
        if (serDataBuffer[1]=="") ser_msg = sendRateVariance();
        else ser_msg = updateRPYRateVariance(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "lin-acc-var") {
        if (serDataBuffer[1]=="") ser_msg = sendAccVariance();
        else ser_msg = updateLinAccVariance(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat(), serDataBuffer[3].toFloat());
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

      else if (serDataBuffer[0] == "rpy-est")
      {
        ser_msg = sendRPY_est();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "heading")
      {
        ser_msg = send_heading();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "reset") {
        ser_msg = resetEEPROM();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      ////////////////////////////////////////////////////
    }
    else
    {
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
  serDataBuffer[3] = "";
}
//////////////////////////////////////////////////////////////////////////















/////////////// I2C COMMUNICATION /////////////////

String i2c_msg = "";

String i2cMsg = "", i2cMsgBuffer, i2cDataBuffer[4];

void i2cSlaveSendData()
{
  String msg = "";
  if (i2c_msg != "")
  {
    msg = i2c_msg;
    i2c_msg = "";
  }
  else
  {
    msg = "0";
    i2c_msg = "";
  }
  char charArray[msg.length() + 1];
  msg.toCharArray(charArray, msg.length() + 1);
  Wire.write(charArray, msg.length());
}

void i2cSlaveReceiveData(int dataSizeInBytes)
{
  int indexPos = 0, i = 0;

  for (int i = 0; i < dataSizeInBytes; i += 1)
  {
    char c = Wire.read();
    i2cMsg += c;
  }

  i2cMsg.trim();

  if (i2cMsg != "")
  {
    do
    {
      indexPos = i2cMsg.indexOf(',');
      if (indexPos != -1)
      {
        i2cMsgBuffer = i2cMsg.substring(0, indexPos);
        i2cMsg = i2cMsg.substring(indexPos + 1, i2cMsg.length());
        i2cDataBuffer[i] = i2cMsgBuffer;
        i2cMsgBuffer = "";
      }
      else
      {
        if (i2cMsg.length() > 0)
          i2cDataBuffer[i] = i2cMsg;
      }
      i += 1;
    } while (indexPos >= 0);
  }

  if (i2cDataBuffer[0] == "rpy-est")
  {
    i2c_msg = sendRPY_est();
  }

  else if (i2cDataBuffer[0] == "heading")
  {
    i2c_msg = send_heading();
  }

  else if (i2cDataBuffer[0] == "rpy-rate")
  {
    i2c_msg = sendRPY_rate();
  }

  else if (i2cDataBuffer[0] == "acc-cal")
  {
    i2c_msg = sendCalAccData();
  }

  i2cMsg = "";
  i2cMsgBuffer = "";
  i2cDataBuffer[0] = "";
  i2cDataBuffer[1] = "";
  i2cDataBuffer[2] = "";
  i2cDataBuffer[3] = "";
}

/////////////////////////////////////////////////////////

#endif