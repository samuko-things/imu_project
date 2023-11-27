# REFERENCE: https://learn.adafruit.com/adafruit-sensorlab-gyroscope-calibration/gyro-calibration-with-jupyter

import matplotlib.pyplot as plt
from collections import deque
import numpy as np


import time
from imu_serial_comm_lib import IMUSerialComm


# portName = '/dev/ttyACM0'
portName = '/dev/ttyUSB0'
imuSer = IMUSerialComm(portName, 115200, 0.1)
time.sleep(5)


# How many sensor samples we want to store
HISTORY_SIZE = 1500

serialport = None


def run_caliberation():
 
    # Deque for axes
    gyro_x = deque(maxlen=HISTORY_SIZE)
    gyro_y = deque(maxlen=HISTORY_SIZE)
    gyro_z = deque(maxlen=HISTORY_SIZE)

    count = 0
    while len(gyro_x) < HISTORY_SIZE:
        try:
            gx, gy, gz = imuSer.get("gyro-raw")

            gyro_x.append(gx)
            gyro_y.append(gy)
            gyro_z.append(gz)

            count +=1
            percent = (count*100)/HISTORY_SIZE
            print("reading_gyro_data...  ", percent, " percent complete")
        except:
            pass


    min_x = min(gyro_x)
    max_x = max(gyro_x)
    min_y = min(gyro_y)
    max_y = max(gyro_y)
    min_z = min(gyro_z)
    max_z = max(gyro_z)

    gx_offset = (max_x + min_x) / 2
    gy_offset = (max_y + min_y) / 2
    gz_offset = (max_z + min_z) / 2

    imuSer.send('gx-off', gx_offset)
    gx_offset = imuSer.get('gx-off')

    imuSer.send('gy-off', gy_offset)
    gy_offset = imuSer.get('gy-off')

    imuSer.send('gz-off', gz_offset)
    gz_offset = imuSer.get('gz-off')

    gyro_calibration = [ gx_offset, gy_offset, gz_offset]
    print("computed gyro offsets in rad/s:", gyro_calibration)


    fig, (uncal, cal) = plt.subplots(2, 1)

    # Clear all axis
    uncal.cla()
    cal.cla()
    t = np.linspace(0, len(gyro_x), len(gyro_x))


    # plot uncalibrated data
    uncal.set_ylim([-1,1])
    uncal.grid(which = "major", linewidth = 0.5)
    uncal.grid(which = "minor", linewidth = 0.2)
    uncal.minorticks_on()

    uncal.plot(t, gyro_x, color='r')
    uncal.plot(t, gyro_y, color='g')
    uncal.plot(t, gyro_z, color='b')
    uncal.title.set_text("Uncalibrated Gyro")
    uncal.set(ylabel='rad/s')

    # plot calibrated data
    cal.set_ylim([-1,1])
    cal.grid(which = "major", linewidth = 0.5)
    cal.grid(which = "minor", linewidth = 0.2)
    cal.minorticks_on()

    cal.plot(t, [x - gyro_calibration[0] for x in gyro_x], color='r')
    cal.plot(t, [y - gyro_calibration[1] for y in gyro_y], color='g')
    cal.plot(t, [z - gyro_calibration[2] for z in gyro_z], color='b')
    cal.title.set_text("Calibrated Gyro")
    cal.set(ylabel='rad/s')

    fig.tight_layout()
    plt.show()





if __name__ == '__main__':
    run_caliberation()














# def get_published_info(data):
#     raw_data = data.data
#     mylist = raw_data.split(',')
    
#     accx = float(mylist[0])
#     accy = float(mylist[1])
#     accz = float(mylist[2])

#     # print(accx, accy, accz)
#     # rospy.loginfo('received: %s', raw_data)


# def subscribing():
#     rospy.init_node(node_name, anonymous=True) #receiver node
#     rospy.Subscriber(sub_topic, String, get_published_info)

# subscribing()