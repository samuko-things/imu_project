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

String sendLinearAcc(){
  String data = String(linear_acc_vect[0], 8);
  data += ",";
  data += String(linear_acc_vect[1], 8);
  data += ",";
  data += String(linear_acc_vect[2], 8);
  return data;
}

String sendRPY_rate(){
  String data = String(roll_rate, 8);
  data += ",";
  data += String(pitch_rate, 8);
  data += ",";
  data += String(yaw_rate, 8);
  return data;
}


String sendRPY_est(){
  String data = String(roll_est, 8);
  data += ",";
  data += String(pitch_est, 8);
  data += ",";
  data += String(yaw_est, 8);
  return data;
}


// String sendRPY_est_with_heading(){
//   String data = String(roll_est, 4);
//   data += ",";
//   data += String(pitch_est, 4);
//   data += ",";
//   data += String(heading_est, 4);
//   return data;
// }

String send_heading(){
  String data = String(heading_est, 8);
  return data;
}


String sendQuternions(){
  String data = String(qw_est, 8);
  data += ",";
  data += String(qx_est, 8);
  data += ",";
  data += String(qy_est, 8);
  data += ",";
  data += String(qz_est, 8);
  return data;
}


// String sendQuternions_with_heading(){
//   String data = String(qwh_est, 4);
//   data += ",";
//   data += String(qxh_est, 4);
//   data += ",";
//   data += String(qyh_est, 4);
//   data += ",";
//   data += String(qzh_est, 4);
//   return data;
// }
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

      if(serDataBuffer[0] == "acc-lin"){
        ser_msg = sendLinearAcc();
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

      // else if(serDataBuffer[0] == "rpy-esth"){
      //   ser_msg = sendRPY_est_with_heading();
      //   Serial.println(ser_msg);
      //   ser_msg = "";
      // }

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

      // else if(serDataBuffer[0] == "quath"){
      //   ser_msg = sendQuternions_with_heading();
      //   Serial.println(ser_msg);
      //   ser_msg = "";
      // }

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