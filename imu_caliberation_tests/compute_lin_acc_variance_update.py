import time
from imu_serial_comm_lib import IMUSerialComm
import numpy as np


# portName = '/dev/ttyACM0'
portName = '/dev/ttyUSB0'
imuSer = IMUSerialComm(portName, 115200, 0.1)
time.sleep(5)

no_of_samples = 1000

ax_lin_arr = []
ay_lin_arr = []
az_lin_arr = []


if __name__ == "__main__":
  for i in range(no_of_samples):
    ax_lin, ay_lin, az_lin = imuSer.get('alin-raw')

    ax_lin_arr.append(ax_lin)
    ay_lin_arr.append(ay_lin)
    az_lin_arr.append(az_lin)

    percent = (i*100)/no_of_samples
    print("reading_gyro_data...  ", percent, " percent complete")

    time.sleep(0.02)
  
  ax_lin_variance = np.var(ax_lin_arr)
  ay_lin_variance = np.var(ay_lin_arr)
  az_lin_variance = np.var(az_lin_arr)


  # imuSer.send('rAng-var', roll_rad_variance)
  # roll_rad_variance = imuSer.get('rAng-var')

  # imuSer.send('pAng-var', pitch_rad_variance)
  # pitch_rad_variance = imuSer.get('pAng-var')

  # imuSer.send('yAng-var', yaw_rad_variance)
  # yaw_rad_variance = imuSer.get('yAng-var')
  

  print('ax_lin_variance =', ax_lin_variance)
  print("")

  print('ay_lin_variance =', ay_lin_variance)
  print("")

  print('az_lin_variance =', az_lin_variance)
  print("")