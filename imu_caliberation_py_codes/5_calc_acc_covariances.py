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

lin_accx_arr = []
lin_accy_arr = []
lin_accz_arr = []


if __name__ == "__main__":
  for i in range(no_of_samples):
    lin_accx, lin_accy, lin_accz = imuSer.get('acc-cal')
    lin_accx_arr.append(lin_accx)
    lin_accy_arr.append(lin_accy)
    lin_accz_arr.append(lin_accz)

    percent = (i*100)/no_of_samples
    print("reading_sensor_data...  ", percent, " percent complete")

    time.sleep(0.02)
  
  lin_accx_variance = np.var(lin_accx_arr)
  lin_accy_variance = np.var(lin_accy_arr)
  lin_accz_variance = np.var(lin_accz_arr)


  imuSer.send('accx-var', lin_accx_variance)
  lin_accx_variance = imuSer.get('accx-var')

  imuSer.send('accy-var', lin_accy_variance)
  lin_accy_variance = imuSer.get('accy-var')

  imuSer.send('accz-var', lin_accz_variance)
  lin_accz_variance = imuSer.get('accz-var')

  

  print("")

  print('lin_accx_variance =', lin_accx_variance)
  print('lin_accy_variance =', lin_accy_variance)
  print('lin_accz_variance =', lin_accz_variance)
  
  print("")