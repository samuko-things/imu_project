#ifndef SERIAL_I2C_COMM_API_H
#define SERIAL_I2C_COMM_API_H
#include<Arduino.h>
#include "global_eeprom_variables.h"








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




String sendRPY_deg(){
  String data = String(roll_deg, 2);
  data += ",";
  data += String(pitch_deg, 2);
  data += ",";
  data += String(yaw_deg, 2);
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

String sendRPY_est(){
  String data = String(roll_est, 4);
  data += ",";
  data += String(pitch_est, 4);
  data += ",";
  data += String(yaw_est, 4);
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
///////////////////////////////////////////////////////////////////////////////////////










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
      if(serDataBuffer[0] == "acc-raw"){
        ser_msg = sendRawAccData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "acc-cal"){
        ser_msg = sendCalAccData();
        Serial.println(ser_msg);
        ser_msg = "";
      }
      
      else if(serDataBuffer[0] == "gyro-raw"){
        ser_msg = sendRawGyroData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "gyro-cal"){
        ser_msg = sendCalGyroData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "mag-raw"){
        ser_msg = sendRawMagData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "mag-cal"){
        ser_msg = sendCalMagData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "rpy-deg"){
        ser_msg = sendRPY_deg();
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

      else if(serDataBuffer[0] == "rpy-est"){
        ser_msg = sendRPY_est();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if(serDataBuffer[0] == "quat"){
        ser_msg = sendQuternions();
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
