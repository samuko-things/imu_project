import matplotlib.pyplot as plt
import matplotlib.animation as animation

import time
from imu_serial_comm_lib import IMUSerialComm



def animate(i):
    global imuSer, axes, axDataList, ayDataList, azDataList, dataPoints

    # ax, ay, az = imuSer.get('aRaw')
    ax, ay, az = imuSer.get('aCal')

    axDataList.append(ax)
    ayDataList.append(ay)
    azDataList.append(az)

    # Fix the list size so that the animation plot 'window' is x number of points
    axDataList = axDataList[-dataPoints:]
    ayDataList = ayDataList[-dataPoints:]
    azDataList = azDataList[-dataPoints:] 
    
    axes.clear()
    axes.plot(axDataList)
    axes.plot(ayDataList)
    axes.plot(azDataList)
    
    axes.grid(which = "major", linewidth = 0.5)
    axes.grid(which = "minor", linewidth = 0.2)
    axes.minorticks_on()

    axes.set_ylim([-20,20]) # Set Y axis limit of plot
    axes.set_title("Accelerometer Data") # Set title of figure
    axes.set_ylabel("accelerition (m/s^2)") # Set title of y axis 
    axes.set_xlabel("number of data points") # Set title of z axis 

    axes.legend(["ax", "ay", "az"], loc ="upper right")






portName = '/dev/ttyACM0'
imuSer = IMUSerialComm(portName, 115200, 0.1)
time.sleep(5)


axDataList = []
ayDataList = []
azDataList = []

dataPoints = 50
                                                        
fig = plt.figure()  # Create Matplotlib plots fig is the 'higher level' plot window
axes = fig.add_subplot(111) # Add subplot to main fig window


if __name__ == '__main__':
  ani = animation.FuncAnimation(fig, animate, frames=100, interval=50) 
  plt.show()
