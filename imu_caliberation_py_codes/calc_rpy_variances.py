import time
from imu_serial_comm_lib import IMUSerialComm
import numpy as np


# portName = '/dev/ttyACM0'
portName = '/dev/ttyUSB0'
imuSer = IMUSerialComm(portName, 115200, 0.1)
time.sleep(5)

no_of_samples = 1000

roll_rad_arr = []
pitch_rad_arr = []
yaw_rad_arr = []

roll_rate_arr = []
pitch_rate_arr = []
yaw_rate_arr = []


if __name__ == "__main__":
  for i in range(no_of_samples):
    r_rad, p_rad, y_rad = imuSer.get('rpy-rad')
    roll_rad_arr.append(r_rad)
    pitch_rad_arr.append(p_rad)
    yaw_rad_arr.append(y_rad)

    r_rate, p_rate, y_rate = imuSer.get('rpy-rate')
    roll_rate_arr.append(r_rate)
    pitch_rate_arr.append(p_rate)
    yaw_rate_arr.append(y_rate)

    percent = (i*100)/no_of_samples
    print("reading_gyro_data...  ", percent, " percent complete")

    time.sleep(0.02)
  
  roll_rad_variance = np.var(roll_rad_arr)
  pitch_rad_variance = np.var(pitch_rad_arr)
  yaw_rad_variance = np.var(yaw_rad_arr)


  roll_rate_variance = np.var(roll_rate_arr)
  pitch_rate_variance = np.var(pitch_rate_arr)
  yaw_rate_variance = np.var(yaw_rate_arr)


  imuSer.send('rAng-var', roll_rad_variance)
  roll_rad_variance = imuSer.get('rAng-var')

  imuSer.send('pAng-var', pitch_rad_variance)
  pitch_rad_variance = imuSer.get('pAng-var')

  imuSer.send('yAng-var', yaw_rad_variance)
  yaw_rad_variance = imuSer.get('yAng-var')


  imuSer.send('rRate-var', roll_rate_variance)
  roll_rate_variance = imuSer.get('rRate-var')

  imuSer.send('pRate-var', pitch_rate_variance)
  pitch_rate_variance = imuSer.get('pRate-var')

  imuSer.send('yRate-var', yaw_rate_variance)
  yaw_rate_variance = imuSer.get('yRate-var')
  

  print('roll_rad_variance =', roll_rad_variance)
  print('roll_rate_variance =', roll_rate_variance)
  print("")

  print('pitch_rad_variance =', pitch_rad_variance)
  print('pitch_rate_variance =', pitch_rate_variance)
  print("")

  print('yaw_rad_variance =', yaw_rad_variance)
  print('yaw_rate_variance =', yaw_rate_variance)
  print("")