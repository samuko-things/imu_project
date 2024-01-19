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


b = np.zeros([3, 1])
A_1 = np.eye(3)
F = 1

magArray = []

mag_x = []
mag_y = []
mag_z = []

calibrated = False

HISTORY_SIZE = 10000

fig, ax = plt.subplots(1, 1)
ax.set_aspect(1)

anim = None

stop = False




def calibrate():
    global b
    global A_1
    global F
    
    # ellipsoid fit
    s = np.array(magArray).T
    M, n, d = __ellipsoid_fit(s)
    
    # calibration parameters
    # note: some implementations of sqrtm return complex type, taking real
    M_1 = linalg.inv(M)
    b = -np.dot(M_1, n)
    A_1 = np.real(F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M))

    ################################################
    b_vect = [[0.0], [0.0], [0.0]]
    A_mat = [[0.0, 0.0, 0.0],[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

    imuSer.send('mag-bvect', b[0][0], b[1][0], b[2][0])
    b_vect[0][0], b_vect[1][0], b_vect[2][0] = imuSer.get('mag-bvect')

    for i in range(3):
      imuSer.send(f'mag-amat{i}', A_1[i][0], A_1[i][1], A_1[i][2])
      A_mat[i][0], A_mat[i][1], A_mat[i][2] = imuSer.get(f'mag-amat{i}')
    ################################################
    
    print()
    print("A_1 matrix:")
    print(A_mat)
    print(A_1)
    print()
    print("b vector:")
    print(b_vect)
    print(b)




def __ellipsoid_fit(s):
    ''' Estimate ellipsoid parameters from a set of points.

        Parameters
        ----------
        s : array_like
          The samples (M,N) where M=3 (x,y,z) and N=number of samples.

        Returns
        -------
        M, n, d : array_like, array_like, float
          The ellipsoid parameters M, n, d.

        References
        ----------
        .. [1] Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
            fitting," in Geometric Modeling and Processing, 2004.
            Proceedings, vol., no., pp.335-340, 2004
    '''

    # D (samples)
    D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                  2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                  2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

    # S, S_11, S_12, S_21, S_22 (eq. 11)
    S = np.dot(D, D.T)
    S_11 = S[:6,:6]
    S_12 = S[:6,6:]
    S_21 = S[6:,:6]
    S_22 = S[6:,6:]

    # C (Eq. 8, k=4)
    C = np.array([[-1,  1,  1,  0,  0,  0],
                  [ 1, -1,  1,  0,  0,  0],
                  [ 1,  1, -1,  0,  0,  0],
                  [ 0,  0,  0, -4,  0,  0],
                  [ 0,  0,  0,  0, -4,  0],
                  [ 0,  0,  0,  0,  0, -4]])

    # v_1 (eq. 15, solution)
    E = np.dot(linalg.inv(C),
                S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

    E_w, E_v = np.linalg.eig(E)

    v_1 = E_v[:, np.argmax(E_w)]
    if v_1[0] < 0: v_1 = -v_1

    # v_2 (eq. 13, solution)
    v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

    # quadric-form parameters
    M = np.array([[v_1[0], v_1[3], v_1[4]],
                  [v_1[3], v_1[1], v_1[5]],
                  [v_1[4], v_1[5], v_1[2]]])
    n = np.array([[v_2[0]],
                  [v_2[1]],
                  [v_2[2]]])
    d = v_2[3]

    return M, n, d




def readCalData():
    global b
    global A_1
    ''' Get a sample.
    
    Returns
    -------
    s : list
    The sample in uT, [x,y,z] (corrected if performed calibration).
    '''

    mx, my, mz = imuSer.get("mag-raw")
    s = np.array([mx,my,mz]).reshape(3, 1)
    s = np.dot(A_1, s - b)
    return s[0,0], s[1,0], s[2,0] #mxCal, myCal, mzCal

    # mxCal, myCal, mzCal = imuSer.get("mag-cal")
    # return mxCal, myCal, mzCal



def onClick(event):   
    global stop
    global calibrated
    global magArray
    global mag_x
    global mag_y
    global mag_z
    
    if stop == False:
        anim.event_source.stop()
        if calibrated == False:
            calibrate()
            mag_x = []
            mag_y = []
            mag_z = []
            magArray = []
            calibrated = True
        stop = True
    else:
        anim.event_source.start()
        stop = False

    
def animate(i):
    global calibrated
    try:
        if calibrated == False:
            mx, my, mz = imuSer.get("mag-raw")
        else:
            # mx, my, mz = readCalData()
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