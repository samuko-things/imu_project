import time
from imu_serial_comm_lib import IMUSerialComm


# portName = '/dev/ttyACM0'
portName = '/dev/ttyUSB0'
imuSer = IMUSerialComm(portName, 115200, 0.1)
time.sleep(5)


if __name__ == "__main__":
  
  isSuccessful = imuSer.send("reset")
  if isSuccessful:
    print("\nSUCCESS: parameters reset successful\nrestart the imu cct to see effect.\n")
  else:
    print("\nERROR: parameters reset not successful\n")