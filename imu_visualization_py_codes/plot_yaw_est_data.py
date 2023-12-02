import matplotlib.pyplot as plt
import matplotlib.animation as animation

import time
from imu_serial_comm_lib import IMUSerialComm



def animate(i):
    global imuSer, unfiltered, filtered, unfilteredDataList, filteredDataList, dataPoints

    roll, pitch, yaw = imuSer.get('rpy-rad')
    roll_est, pitch_est, yaw_est = imuSer.get('rpy-est')

    unfilteredDataList.append(yaw)
    filteredDataList.append(yaw_est)

    # Fix the list size so that the animation plot 'window' is x number of points
    unfilteredDataList = unfilteredDataList[-dataPoints:] 
    filteredDataList = filteredDataList[-dataPoints:] 
    
    axes.clear()
    axes.plot(unfilteredDataList)
    axes.plot(filteredDataList)
    
    axes.grid(which = "major", linewidth = 0.5)
    axes.grid(which = "minor", linewidth = 0.2)
    axes.minorticks_on()

    axes.set_ylim([-5,5]) # Set Y axis limit of plot
    axes.set_title(f"yaw_unfiltered: {yaw}\nyaw_filtered: {yaw_est}") # Set title of figure
    axes.set_ylabel("angular pos (radians)") # Set title of y axis 
    axes.set_xlabel("number of data points") # Set title of z axis 

    axes.legend(["unfiltered yaw", "filtered yaw"], loc ="upper right")






# portName = '/dev/ttyACM0'
portName = '/dev/ttyUSB0'
imuSer = IMUSerialComm(portName, 115200, 0.1)
for i in range(30):
  time.sleep(1.0)
  print(i+1, " sec")
# time.sleep(30)


unfilteredDataList = []
filteredDataList = []

dataPoints = 50

fig = plt.figure()  # Create Matplotlib plots fig is the 'higher level' plot window
axes = fig.add_subplot(111) # Add subplot to main fig window


if __name__ == '__main__':
  ani = animation.FuncAnimation(fig, animate, frames=100, interval=50) 
  fig.tight_layout()
  plt.show()
