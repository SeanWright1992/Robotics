
import math
import numpy as np

class IK(object):
    def __init__(self, Robot,x,y,z):
        a1=6.9
        a2=14.5
        a3=18.6
        a4=7.5

        self.theta1 = np.degrees(np.arctan2(y,x))
        working_x= x
        working_y= y
        working_z= z+7.2827

        r= np.sqrt(working_x**2+working_y**2)
        s=working_z-a1

        D=(r**2+s**2-a2**2-a3**2)/(2*a2*a3)

        self.theta3 = np.degrees(np.arctan2(-math.sqrt(1-D**2),D))
        #theta2 = np.degrees(np.arctan2(s,r))- np.degrees(np.arctan2((a3*np.degrees(np.sin((theta3),a2+a3*np.degrees(np.cos(theta3)))))))
        part1 = np.degrees(np.arctan2(s,r))
        part2 = np.degrees(np.arctan2(a3*np.degrees(np.sin(self.theta3)), a2+a3*np.degrees(np.cos(self.theta3))))
        self.theta2 = part1 - part2
        self.theta4 = -90.0 - self.theta2 - self.theta3
        print(self.theta1, self.theta2, self.theta3, self.theta4)

        #self.pwm1 = round(-9.375*theta1 + 1500)
        #self.pwm2 = round(8.2418*theta2 + 750)
        #self.pwm3 = round(-8.8235*theta3 + 750)
        #self.pwm4 = round(9.375*theta4 + 1500)

        #self.pwm1 = round(-9.375*theta1 + 1500)
        #self.pwm2 = round(8.2418*theta2 + 750)
        #self.pwm3 = round(-8.8235*theta3 + 750)
        #self.pwm4 = round(9.375*theta4 + 1500)

        #print(self.pwm1)
        #print(self.pwm2)
        #print(self.pwm3)
        #print(self.pwm4)
    def PWMValues(self):

        #Angles = [self.pwm1, self.pwm2, self.pwm3, self.pwm4]
        Angles = [self.theta1, self.theta2, self.theta3, self.theta4]
        return (Angles)