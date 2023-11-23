import matplotlib.pyplot as plt
import matplotlib.animation as animation

import time
from imu_serial_comm_lib import IMUSerialComm



def animate(i):
    global imuSer, axes, gxDataList, gyDataList, gzDataList, dataPoints

    # gx, gy, gz = imuSer.get('gyro-raw')
    gx, gy, gz = imuSer.get('gyro-cal')

    gxDataList.append(gx)
    gyDataList.append(gy)
    gzDataList.append(gz)

    # Fix the list size so that the animation plot 'window' is x number of points
    gxDataList = gxDataList[-dataPoints:]
    gyDataList = gyDataList[-dataPoints:]
    gzDataList = gzDataList[-dataPoints:] 
    
    axes.clear()
    axes.plot(gxDataList)
    axes.plot(gyDataList)
    axes.plot(gzDataList)
    
    axes.grid(which = "major", linewidth = 0.5)
    axes.grid(which = "minor", linewidth = 0.2)
    axes.minorticks_on()

    axes.set_ylim([-5,5]) # Set Y axis limit of plot
    axes.set_title("Gyroscope Data") # Set title of figure
    axes.set_ylabel("angular rate (rad/s^2)") # Set title of y axis 
    axes.set_xlabel("number of data points") # Set title of z axis 

    axes.legend(["gx", "gy", "gz"], loc ="upper right")






portName = '/dev/ttyACM0'
imuSer = IMUSerialComm(portName, 115200, 0.1)
time.sleep(5)


gxDataList = []
gyDataList = []
gzDataList = []

dataPoints = 50
                                                        
fig = plt.figure()  # Create Matplotlib plots fig is the 'higher level' plot window
axes = fig.add_subplot(111) # Add subplot to main fig window


if __name__ == '__main__':
  ani = animation.FuncAnimation(fig, animate, frames=100, interval=50) 
  plt.show()
