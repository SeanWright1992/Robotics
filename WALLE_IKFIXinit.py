import numpy as np
import math
import pandas as pd
import time
import serial
import cv2

class robot(object):
    def __init__(self):

        self.WALLE = serial.Serial(
            port='com5',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )  # configure the serial connections (the parameters differs on the device you are connecting to)

        #self.WALLE.write(b'hello')  # write a string
        #self.WALLE.close()  # close port

        print("About to move to default position")
        self.DefaultPOSDown()
        #need to move arm away from camera

        time.sleep(2)

        print("Initialize empty list to hold angles")
        self.AngleList = [0]*4


        print("Run inversekinematics for x,y,z coords")
        #Need to run OpenCV code here to find the x,y,z coordinates in the image.
        #need to save the color and position

        #This part of the inverse kinematics will move to the object
        self.InverseKinematicsDown(-10, 20, 5)
        self.SetAngles(self.AngleList,2000)


        time.sleep(10)
        #this part of the code will pick up the object, check its color, and dump it at corresponding box

        #self.locale = "blue"
        #self.GrabAndPlace(self.locale)
        #time.sleep(2)



        #self.WALLE.close()

    def DefaultPOSDown(self):
        #end effector down at 90 degrees
        self.WALLE.write(b"#0 P1530 #1 P1540 #2 P1440 #3 P750 #4 P750\r")


    def InverseKinematicsDown(self, x,y,z):
        do = 7.5
        L = 14.5
        M = 18#19.4275
        N = 5.5725
        #N = 8.5725;

        R = math.sqrt(x**2 + y**2)
        #s = R - N
        Q = math.sqrt((z-do+N)**2 + (R)**2)

        angleF = np.degrees(np.arctan2(z - do + N, R))

        angleG = np.degrees(np.arccos((L**2 + Q**2 - M**2) / (2 * L * Q)))

        angleA = angleF + angleG

        angleB = np.degrees(np.arccos(((M**2 + L**2 - Q**2) / (2 * L * M))))

        angleC = -angleB - angleA + 2 * math.pi

        theta1 = (np.degrees(np.arctan2(y, x))) - 90
        theta2 = (angleA) -90
        theta3 = (angleB) -90
        theta4 = (-theta2 - theta3) -90 - 19.75

        print('theta 1 is {}'.format(theta1))
        print('theta 2 is {}'.format(theta2))
        print('theta 3 is {}'.format(theta3))
        print('theta 4 is {}'.format(theta4))

        self.AngleList = [theta1, theta2, theta3, theta4]

    def SetAngles(self, Angles, time):
        #self.WALLE.write(b"#0 P1530 #1 P1540 #2 P1440 #3 P1465\r")


        ch0 = (1540 - 10.72 * Angles[0]) # 10.72
        ch1 = (1470 + 8.21 * Angles[1]) # 8.21 #1470
        ch2 = (1440 - 8.18 * Angles[2]) # 8.18
        ch3 = (1500 + 8.96296 * Angles[3])

        #ch0 1530, 1540,1440, 1500

        #current settings that work well 1500, 1500, 1440, 1500

        out = ("#0 P{} T{} #1 P{} T{} #2 P{} #3 P{}\r".format(ch0,time, ch1, time, ch2, ch3))
        self.WALLE.write(bytes(out, 'utf8'))

    def GrabAndPlace(self, locale):
        print("grabbing and placing object")

        #closes gripper to grab box
        self.WALLE.write(b" #4 P2250\r")
        if locale == "blue":
            print("moving")
            self.InverseKinematicsDown(10,20,5)
            self.SetAngles(self.AngleList, 2000)
            time.sleep(8)
        else:
            print("uhmmm what?")

        #let go of block
        self.WALLE.write(b" #4 P750\r")

        self.DefaultPOSDown()



    def DefaultPOS(self):
        #returns WALLE to its original position
        #end effector up at 0 degrees parallel with a2
        self.WALLE.write(b"#0 P1530 #1 P1540 #2 P1440 #3 P1465\r")

    """
       def InverseKinematics(self, x,y,z):
           do = 7.5
           L = 15
           M = 19.4275
           N = 6.5725
           #N = 8.5725;

           R = math.sqrt(x**2 + y**2)
           s = R - N
           Q = math.sqrt(s**2 + (z - do)**2)

           angleF = np.degrees(np.arctan2(z - do, s))

           angleG = np.degrees(np.arccos((L**2 + Q**2 - M**2) / (2 * L * Q)))

           angleA = angleF + angleG

           angleB = np.degrees(np.arccos(((M**2 + L**2 - Q**2) / (2 * L * M))))

           angleC = -angleB - angleA + 2 * math.pi

           theta1 = (np.degrees(np.arctan2(y, x))) - 90
           theta2 = (angleA) -90
           theta3 = (angleB) -90
           theta4 = (-theta2 - theta3)-90

           print('theta 1 is {}'.format(theta1))
           print('theta 2 is {}'.format(theta2))
           print('theta 3 is {}'.format(theta3))
           print('theta 4 is {}'.format(theta4))


           self.AngleList = [theta1, theta2, theta3, theta4]
       """









Startup = robot()










