#REFERENCE_1: https://teslabs.com/articles/magnetometer-calibration/
#REFERENCE_2: https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration/magnetic-calibration-with-jupyter

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from scipy import linalg

import time
from imu_serial_comm_lib import IMUSerialComm


# portName = '/dev/ttyACM0'
portName = '/dev/ttyUSB0'
imuSer = IMUSerialComm(portName, 115200, 0.1)
time.sleep(5)


# How many sensor samples we want to store
HISTORY_SIZE = 10000

magArray = []

mag_x = []
mag_y = []
mag_z = []

HISTORY_SIZE = 10000

fig, ax = plt.subplots(1, 1)
ax.set_aspect(1)

anim = None

stop = False




def onClick(event):   
    global stop
    
    if stop == False:
        anim.event_source.stop()
        stop = True
    else:
        anim.event_source.start()
        stop = False

    
def animate(i):
    global calibrated
    try:
        mx, my, mz = imuSer.get("mag-cal")
        
        magArray.append([mx,my,mz])
        mag_x.append(mx)
        mag_y.append(my)
        mag_z.append(mz)

        # Clear all axis
        ax.cla()

        # Display the sub-plots
        ax.scatter(mag_x, mag_y, color='r')
        ax.scatter(mag_y, mag_z, color='g')
        ax.scatter(mag_z, mag_x, color='b')
        
        if len(mag_x) == HISTORY_SIZE:
            anim.event_source.stop()
    ##    # Pause the plot for INTERVAL seconds 
    ##    plt.pause(INTERVAL)
    except:
        pass




if __name__ == "__main__":
  fig.canvas.mpl_connect('button_press_event', onClick)    
  anim = FuncAnimation(fig, animate, frames = np.arange(0, 10000, 1), interval=50)

  plt.show()