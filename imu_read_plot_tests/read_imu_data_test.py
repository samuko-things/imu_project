import time
from imu_serial_comm_lib import IMUSerialComm


# portName = '/dev/ttyACM0'
portName = '/dev/ttyUSB0'
imuSer = IMUSerialComm(portName, 115200, 0.1)
time.sleep(5)


if __name__ == "__main__":
  while True:
    # ax, ay, az = imuSer.get('acc-cal')
    # print(ax,ay,az)

    # gx, gy, gz = imuSer.get('gyro-cal')
    # print(gx,gy,gz)

    mx, my, mz = imuSer.get('mag-cal')
    print(mx,my,mz)

    # r_deg, p_deg, y_deg = imuSer.get('rpy-deg')
    # print(r_deg, p_deg, y_deg)

    # r_rad, p_rad, y_rad = imuSer.get('rpy-rad')
    # print(r_rad, p_rad, y_rad)

    # r_rate, p_rate, y_rate = imuSer.get('rpy-rate')
    # print(r_rate, p_rate, y_rate)

    # r_est, p_est, y_est = imuSer.get('rpy-est')
    # print(r_est, p_est, y_est)
    # print("")

    # qw, qx, qy, qz = imuSer.get('quat')
    # print(qw, qx, qy, qz)
    # print("")

    print("")

    time.sleep(0.02)