#!/usr/bin/env python
''' Python file for running Postion Controller '''

# Importing the required libraries
import time
from time import sleep
from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rospy



class Edrone():
    """ Edrone Class """

    def __init__(self):

        rospy.init_node('position_controller')  # Initializing ros node with name position_controller

        # Initializing Altitude ,Latitude and Longitude values

        self.altitude_coord = 0.0
        self.latitude_coord = 0.0
        self.longitude_coord = 0.0

        # Initalinzing Altitude ,Latitude and Longitude Error
        # Index 0 for Altitude , Index 1 for Latitude , Index 2 for Longitude error

        self.err = [0.0, 0, 0]

        # Seting Setpoint 1 Coordinates

        self.setpoint = [3, 19, 72]

        # Setting Kp, Ki, Kd values for altitude, latitude, longitude with mentioned indexing order

        self.k_p = [420, 4750, 4750]
        self.k_i = [0.032, 560, 560]
        self.k_d = [219, 36000, 36000]

        # Initializing Error Values for altitude, latitude, longitude with mentioned indexing order

        self.lasterror = [0, 0, 0]
        self.errorsum = [0, 0, 0]
        self.output = [0, 0, 0]
        self.changerror = [0, 0, 0]

        # Declaring edrone_cmd and initialing Values

        self.drone_cmd = edrone_cmd()
        self.drone_cmd.rcThrottle = 0.0
        self.drone_cmd.rcRoll = 0.0
        self.drone_cmd.rcPitch = 0.0
        self.drone_cmd.rcYaw = 0.0
        self.setpoint_one_reached = 0
        self.sample_time = 0.06

        # Declaring Publisher to topic drone_command for input to node attitude_controller.py

        self.drone_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        # Subscribing to /edrone/gps to get cuurent altitude, latitude, longitude coordinates

        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps)

        # We read the msg and get the cuurent coordinates of the Edrone

    def gps(self, msg):
        """ gps runs each time coordinates are published to topic /edrone/gps """
        self.latitude_coord = msg.latitude
        self.longitude_coord = msg.longitude
        self.altitude_coord = msg.altitude


    def pid(self):
        """  Main PID Controller for altitude, latitude, longitude Error """

        # Calculating Error for altitude, latitude, longitude

        self.err[0] = self.setpoint[0] - self.altitude_coord
        self.err[1] = self.setpoint[1] - self.latitude_coord
        self.err[2] = self.setpoint[2] - self.longitude_coord

        # Calculating Change in Error for altitude, latitude, longitude

        self.changerror[0] = self.err[0] - self.lasterror[0]
        self.changerror[1] = self.err[1] - self.lasterror[1]
        self.changerror[2] = self.err[2] - self.lasterror[2]

        # Calculating sum of Error for altitude, latitude, longitude

        self.errorsum[0] = (self.errorsum[0] + self.err[0])*self.sample_time
        self.errorsum[1] = (self.errorsum[1] + self.err[1])*self.sample_time
        self.errorsum[2] = (self.errorsum[2] + self.err[2])*self.sample_time

        # Calculating Output which is to be sent to attitude_controller.py through edrone/cmd pub

        self.output[0] = self.k_p[0]*self.err[0] + self.k_i[0]*self.errorsum[0] + self.k_d[0]*self.changerror[0]/self.sample_time
        self.output[1] = self.k_p[1]*self.err[1] + self.k_i[1]*self.errorsum[1] + self.k_d[1]*self.changerror[1]/self.sample_time
        self.output[2] = self.k_p[2]*self.err[2] + self.k_i[2]*self.errorsum[2] + self.k_d[2]*self.changerror[2]/self.sample_time

        # Equation for Throttle , Pitch, Roll and Yaw values for attitude_controller.py

        self.drone_cmd = edrone_cmd()
        self.drone_cmd.rcThrottle = self.output[0] + 1500
        self.drone_cmd.rcPitch = 1500 + 5.15*self.output[1]
        self.drone_cmd.rcRoll = 1500 + 5.15*self.output[2]
        self.drone_cmd.rcYaw = 1500

        # Storing Current error

        self.lasterror[0] = self.err[0]
        self.lasterror[1] = self.err[1]
        self.lasterror[2] = self.err[2]


        # Check to see If we have reached Setpoint 1 within allowed error range

        if 2.99 < self.altitude_coord < 3.01 and 18.999995483 < self.latitude_coord < 19.000004517 and 71.999995251 < self.longitude_coord < 72.000004749:
            self.setpoint[1] = 19.0000451704  # We Set Setpoint values to the coord of Setpoint 2
            self.setpoint[2] = 72             # Edrone Now moves towards Setpoint 2
            self.setpoint_one_reached = 1

        # If we have reached Setpoint 1, we check if the Edrone has reached towards Setpoint 2
        # We check if are within allowable error range

        if 2.99 < self.altitude_coord < 3.01 and 19.000040653 < self.latitude_coord < 19.000049687 and 71.999995251 < self.longitude_coord < 72.000004749 and 0 < abs(self.changerror[1]) < 0.00000005 and self.setpoint_one_reached == 1:
            self.setpoint[0] = 0.31           # We Set Setpoint Values to coord of Setpoint 3
            self.setpoint[1] = 19.0000451704  # Edrone Now moves towards final setpoint,the target
            self.setpoint[2] = 72

        # Publishing msg drone_cmd

        self.drone_pub.publish(self.drone_cmd)
        print self.setpoint
        print self.latitude_coord


if __name__ == '__main__':
    time.sleep(2.5)
    E_DRONE = Edrone()
    R = rospy.Rate(16.67)
    while not rospy.is_shutdown():
        E_DRONE.pid()
        R.sleep()
