import matplotlib.pyplot as plt
from collections import deque
import numpy as np

import time
from imu_serial_comm_lib import IMUSerialComm


portName = '/dev/ttyACM0'
imuSer = IMUSerialComm(portName, 115200, 0.1)
time.sleep(5)

# How many sensor samples we want to store
HISTORY_SIZE = 1500

serialport = None


def average(val):
    ans = 0
    for i in val:
      ans= ans + i
    
    ans = ans/len(val)
    
    return ans


def run_caliberation():
 
    # Deque for axes
    acc_x = deque(maxlen=HISTORY_SIZE)
    acc_y = deque(maxlen=HISTORY_SIZE)
    acc_z = deque(maxlen=HISTORY_SIZE)

    count = 0
    while len(acc_x) < HISTORY_SIZE:
        try:
            ax, ay, az = imuSer.get("aRaw")

            acc_x.append(ax)
            acc_y.append(ay)
            acc_z.append(az)

            count +=1
            percent = (count*100)/HISTORY_SIZE
            print("reading_acc_data...  ", percent, " percent complete")
        except:
            pass

    acc_calibration = [ average(acc_x), average(acc_y), (average(acc_z) + 9.8)]
    print("computed acc offsets in g:", acc_calibration)


    fig, (uncal, cal) = plt.subplots(nrows=2)

    # Clear all axis
    uncal.cla()
    cal.cla()
    t = np.linspace(0, len(acc_x), len(acc_x))


    # plot uncalibrated data
    uncal.plot(t, acc_x, color='r')
    uncal.plot(t, acc_y, color='g')
    uncal.plot(t, acc_z, color='b')
    uncal.title.set_text("Uncalibrated Acc")
    uncal.set(ylabel='g')

    uncal.grid(which = "major", linewidth = 0.5)
    uncal.grid(which = "minor", linewidth = 0.2)
    uncal.minorticks_on()


    # plot calibrated data
    cal.plot(t, [x - acc_calibration[0] for x in acc_x], color='r')
    cal.plot(t, [y - acc_calibration[1] for y in acc_y], color='g')
    cal.plot(t, [z - acc_calibration[2] for z in acc_z], color='b')
    cal.title.set_text("Calibrated Acc")
    cal.set(ylabel='g')

    cal.grid(which = "major", linewidth = 0.5)
    cal.grid(which = "minor", linewidth = 0.2)
    cal.minorticks_on()



    fig.tight_layout()
    plt.show()



if __name__ == '__main__':
    run_caliberation()

