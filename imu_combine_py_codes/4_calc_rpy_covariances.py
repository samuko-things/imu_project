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
    r_rad, p_rad, y_rad = imuSer.get('rpy-rad')
    roll_rad_arr.append(r_rad)
    pitch_rad_arr.append(p_rad)
    yaw_rad_arr.append(y_rad)

    r_rate, p_rate, y_rate = imuSer.get('rpy-rate')
    roll_rate_arr.append(r_rate)
    pitch_rate_arr.append(p_rate)
    yaw_rate_arr.append(y_rate)

    lin_accx, lin_accy, lin_accz = imuSer.get('acc-cal')
    lin_accx_arr.append(lin_accx)
    lin_accy_arr.append(lin_accy)
    lin_accz_arr.append(lin_accz)

    percent = (i*100)/no_of_samples
    print("reading_sensor_data...  ", percent, " percent complete")

    time.sleep(0.02)
  
  roll_rad_variance = np.var(roll_rad_arr)
  pitch_rad_variance = np.var(pitch_rad_arr)
  yaw_rad_variance = np.var(yaw_rad_arr)


  roll_rate_variance = np.var(roll_rate_arr)
  pitch_rate_variance = np.var(pitch_rate_arr)
  yaw_rate_variance = np.var(yaw_rate_arr)


  lin_accx_variance = np.var(lin_accx_arr)
  lin_accy_variance = np.var(lin_accy_arr)
  lin_accz_variance = np.var(lin_accz_arr)

  print("")

  print('computed roll_angle_variance =', roll_rad_variance)
  print('computed pitch_angle_variance =', pitch_rad_variance)
  print('computed yaw_angle_variance =', yaw_rad_variance)
  print("")

  print('computed roll_rate_variance =', roll_rate_variance)
  print('computed pitch_rate_variance =', pitch_rate_variance)
  print('computed yaw_rate_variance =', yaw_rate_variance)
  print("")

  print('computed lin_accx_variance =', lin_accx_variance)
  print('computed lin_accy_variance =', lin_accy_variance)
  print('computed lin_accz_variance =', lin_accz_variance)


  imuSer.send('rpy-ang-var', roll_rad_variance, pitch_rad_variance, yaw_rad_variance)
  roll_rad_variance, pitch_rad_variance, yaw_rad_variance = imuSer.get('rpy-ang-var')

  imuSer.send('rpy-rate-var', roll_rate_variance, pitch_rate_variance, yaw_rate_variance)
  roll_rate_variance, pitch_rate_variance, yaw_rate_variance = imuSer.get('rpy-rate-var')

  imuSer.send('lin-acc-var', lin_accx_variance, lin_accy_variance, lin_accz_variance)
  lin_accx_variance, lin_accy_variance, lin_accz_variance = imuSer.get('lin-acc-var')

  
  print("")

  print('stored roll_angle_variance =', roll_rad_variance)
  print('stored pitch_angle_variance =', pitch_rad_variance)
  print('stored yaw_angle_variance =', yaw_rad_variance)
  print("")

  print('stored roll_rate_variance =', roll_rate_variance)
  print('stored pitch_rate_variance =', pitch_rate_variance)
  print('stored yaw_rate_variance =', yaw_rate_variance)
  print("")

  print('stored lin_accx_variance =', lin_accx_variance)
  print('stored lin_accy_variance =', lin_accy_variance)
  print('stored lin_accz_variance =', lin_accz_variance)
  
  print("")