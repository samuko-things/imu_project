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


String sendRPY_est(){
  String data = String(roll_est, 4);
  data += ",";
  data += String(pitch_est, 4);
  data += ",";
  data += String(yaw_est, 4);
  return data;
}


String sendRPY_est_with_heading(){
  String data = String(roll_est, 4);
  data += ",";
  data += String(pitch_est, 4);
  data += ",";
  data += String(heading, 4);
  return data;
}

String send_heading(){
  String data = String(heading, 4);
  data += ",";
  data += String(heading_deg, 2);
  return data;
}


String sendQuternions(){
  String data = String(qw, 4);
  data += ",";
  data += String(qx, 4);
  data += ",";
  data += String(qy, 4);
  data += ",";
  data += String(qz, 4);
  return data;
}


String sendQuternions_with_heading(){
  String data = String(qwh, 4);
  data += ",";
  data += String(qxh, 4);
  data += ",";
  data += String(qyh, 4);
  data += ",";
  data += String(qzh, 4);
  return data;
}
///////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////
String sendRollAngleVariance(){
  return String(R_roll,10);
}

String sendPitchAngleVariance(){
  return String(R_pitch,10);
}

String sendYawAngleVariance(){
  return String(R_yaw,10);
}
//////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////
String sendRollRateVariance(){
  return String(Q_roll,10);
}

String sendPitchRateVariance(){
  return String(Q_pitch,10);
}

String sendYawRateVariance(){
  return String(Q_yaw,10);
}
//////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////
String sendAccxVariance(){
  return String(accx_variance,10);
}

String sendAccyVariance(){
  return String(accy_variance,10);
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

      if(serDataBuffer[0] == "rpy-rad"){
        ser_msg = sendRPY_rad();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "rpy-rate"){
        ser_msg = sendRPY_rate();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "rpy-est"){
        ser_msg = sendRPY_est();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "rpy-esth"){
        ser_msg = sendRPY_est_with_heading();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "heading"){
        ser_msg = send_heading();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "quat"){
        ser_msg = sendQuternions();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "quath"){
        ser_msg = sendQuternions_with_heading();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "rAng-var") {
        ser_msg = sendRollAngleVariance();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "rRate-var") {
        ser_msg = sendRollRateVariance();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "pAng-var") {
        ser_msg = sendPitchAngleVariance();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "pRate-var") {
        ser_msg = sendPitchRateVariance();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "yAng-var") {
        ser_msg = sendYawAngleVariance();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "yRate-var") {
        ser_msg = sendYawRateVariance();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "accx-var") {
        ser_msg = sendAccxVariance();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "accy-var") {
        ser_msg = sendAccyVariance();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "accz-var") {
        ser_msg = sendAcczVariance();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      ////////////////////////////////////////////////////

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