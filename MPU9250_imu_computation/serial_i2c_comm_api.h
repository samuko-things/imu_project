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

/////////////////////////////////////////////////////////////////////////////////////

String sendLinearAcc()
{
  String data = String(linear_acc_vect[0], 6);
  data += ",";
  data += String(linear_acc_vect[1], 6);
  data += ",";
  data += String(linear_acc_vect[2], 6);
  return data;
}

String sendRPY_rate()
{
  String data = String(roll_rate, 6);
  data += ",";
  data += String(pitch_rate, 6);
  data += ",";
  data += String(yaw_rate, 6);
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
String sendRollRateVariance()
{
  return String(Q_roll, 6);
}

String sendPitchRateVariance()
{
  return String(Q_pitch, 6);
}

String sendYawRateVariance()
{
  return String(Q_yaw, 6);
}
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
String sendAccxVariance()
{
  return String(accx_variance, 6);
}

String sendAccyVariance()
{
  return String(accy_variance, 6);
}

String sendAcczVariance()
{
  return String(accz_variance, 6);
}
////////////////////////////////////////////////////////////////////






///////////////// SERIAL COMMUNICATION //////////////////////
String ser_msg = "";

String serMsg = "", serMsgBuffer, serDataBuffer[3];

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

      if (serDataBuffer[0] == "acc-lin")
      {
        ser_msg = sendLinearAcc();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "rpy-rate")
      {
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

      else if (serDataBuffer[0] == "rRate-var")
      {
        ser_msg = sendRollRateVariance();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "pRate-var")
      {
        ser_msg = sendPitchRateVariance();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "yRate-var")
      {
        ser_msg = sendYawRateVariance();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "accx-var")
      {
        ser_msg = sendAccxVariance();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "accy-var")
      {
        ser_msg = sendAccyVariance();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "accz-var")
      {
        ser_msg = sendAcczVariance();
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

  if (i2cDataBuffer[0] == "acc-lin")
  {
    i2c_msg = sendLinearAcc();
  }

  else if (i2cDataBuffer[0] == "rpy-rate")
  {
    i2c_msg = sendRPY_rate();
  }

  else if (i2cDataBuffer[0] == "rpy-est")
  {
    i2c_msg = sendRPY_est();
  }

  else if (i2cDataBuffer[0] == "heading")
  {
    i2c_msg = send_heading();
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