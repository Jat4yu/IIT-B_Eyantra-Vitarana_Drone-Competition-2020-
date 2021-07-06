#!/usr/bin/env python
''' Python file for running Postion Controller '''

# Importing the required libraries
import time
from sensor_msgs.msg import LaserScan
from time import sleep
from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest
from std_msgs.msg import String
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
        self.gripper = ''
        self.str = 'True'
        self.bool = False
        self.qr_data = []
        self.start = 1

        # Initalinzing Altitude ,Latitude and Longitude Error
        # Index 0 for Altitude , Index 1 for Latitude , Index 2 for Longitude error

        self.err = [0.0, 0, 0]
        self.allranges = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Seting Setpoint 1 Coordinates

        self.setpoint = [24.16, 19.0009248718, 71.9998318945]

        # Setting Kp, Ki, Kd values for altitude, latitude, longitude with mentioned indexing order

        self.k_p = [420, 5750, 5750]
        self.k_i = [0.032, 560, 560]
        self.k_d = [219, 38000, 38000]

        # Initializing Error Values for altitude, latitude, longitude with mentioned indexing order

        self.lasterror = [0, 0, 0]
        self.errorsum = [0, 0, 0]
        self.output = [0, 0, 0]
        self.changerror = [0, 0, 0]

        # Declaring edrone_cmd and initialing Values

        self.longitude_right_range = False
        self.longitude_left_range = False
        self.latitude_fwd_range = False
        self.latitude_back_range = False
        self.logger = 0.0
        self.save = False
        self.drone_cmd = edrone_cmd()
        self.drone_cmd.rcThrottle = 0.0
        self.drone_cmd.rcRoll = 0.0
        self.drone_cmd.rcPitch = 0.0
        self.drone_cmd.rcYaw = 0.0
        self.setpoint_one_reached = 0
        self.setpoint22 = 0
        self.setpoint2 = 0
        self.setpoint3 = 0
        self.sample_time = 0.06
        self.setpoint4 = 0
        self.setpoint5 = 0

        # Declaring Publisher to topic drone_command for input to node attitude_controller.py

        self.drone_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        # Subscribing to /edrone/gps to get cuurent altitude, latitude, longitude coordinates
        rospy.Subscriber('/edrone/gripper_check', String, self.check)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps)
        rospy.Subscriber('/qr_info', QR_scanned, self.qr_receiver)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.rangefinder)

        # We read the msg and get the cuurent coordinates of the Edrone

    def rangefinder(self, msg):
        """ To Get all the ranges from range_finder_top """
        self.allranges = msg.ranges

    def gps(self, msg):
        """ gps runs each time coordinates are published to topic /edrone/gps """
        self.latitude_coord = msg.latitude
        self.longitude_coord = msg.longitude
        self.altitude_coord = msg.altitude

    def check(self, msg):
        """ To check if gripper can be activated """
        self.gripper = msg.data

    def Gripper_client(self, msg):
        """ Gripper Client Handler"""
        rospy.wait_for_service('/edrone/activate_gripper')
        try:
            service = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
            resp = service(msg)
            return resp.result
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e

    def qr_receiver(self, msg):
        """ QR code Reciever """
        self.qr_data = map(float, msg.detected.split(","))



    def pid(self):
        """  Main PID Controller for altitude, latitude, longitude Error """

        def range_check(self, arglist):
            """ To check the ranges and return True if range < 17 """
            if type(arglist[0]) == float:
                if 1 < arglist[0] < 17.0:
                    self.longitude_right_range = True
                else:
                    self.longitude_right_range = False

            if type(arglist[1]) == float:
                if 1 < arglist[1] < 17.0:
                    self.latitude_back_range = True
                else:
                    self.latitude_back_range = False

            if type(arglist[2]) == float:
                if 1 < arglist[2] < 17.0:
                    self.longitude_left_range = True
                else:
                    self.longitude_left_range = False


            if type(arglist[3]) == float:
                if 1 < arglist[3] < 17.0:
                    self.latitude_fwd_range = True
                else:
                    self.latitude_fwd_range = False
            return [self.longitude_right_range, self.latitude_back_range, self.longitude_left_range, self.latitude_fwd_range]
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
        self.drone_cmd.rcPitch = 1500 + 6*self.output[1]
        self.drone_cmd.rcRoll = 1500 + 6*self.output[2]
        self.drone_cmd.rcYaw = 1500

        # Storing Current error

        self.lasterror[0] = self.err[0]
        self.lasterror[1] = self.err[1]
        self.lasterror[2] = self.err[2]

        # Check to see If we have reached Setpoint 1 within allowed error range

        if self.start == 1 and abs(self.lasterror[0]) < 0.1:
            self.setpoint[0] = 24.1599967919
            self.setpoint[1] = 19.0007046575  # We Set Setpoint values to the coord of Setpoint 2
            self.setpoint[2] = 71.9998955286           # Edrone Now moves towards Setpoint 2
            self.setpoint_one_reached = 1
            print "Set"
            self.start = 0

        if self.setpoint_one_reached == 1 and 19.000700000 < self.latitude_coord < 19.000709 and 71.999892 < self.longitude_coord < 71.9999:
            self.setpoint[0] = 22.17
            self.setpoint[1] = 19.0007046575  # We Set Setpoint values to the coord of Setpoint 2
            self.setpoint[2] = 71.9998955286           # Edrone Now moves towards Setpoint 2
            self.setpoint22 = 1


        if self.setpoint22 == 1 and abs(self.lasterror[1]) < 0.0000004  and abs(self.lasterror[2]) < 0.0000004:
            if self.gripper == self.str:
                self.bool = True
                if self.Gripper_client(self.bool):    # If We get a "True" Response from Gripper then we can proceed
                    self.k_p[1] = 2000
                    self.k_p[2] = 2000
                    self.k_i[1] = 2637
                    self.k_i[2] = 2637
                    self.k_d[1] = 16700
                    self.k_d[2] = 12000
                    self.setpoint2 = 1

        if self.setpoint2 == 1 and len(self.qr_data) > 1:
            self.setpoint[0] = 24.16         # We Set Setpoint Values to coord of Setpoint 3
            self.setpoint[1] = self.latitude_coord  # Edrone Now moves towards final setpoint,the target
            self.setpoint[2] = self.longitude_coord
            self.setpoint3 = 1

        if self.setpoint3 == 1 and abs(self.lasterror[0]) < 0.1:
            if range_check(self, self.allranges)[3]:             # If eDrone in near on obstacle from the front then we get True resp
                if not self.save:                                # Hence we avoid the obstacle by changing setpoint
                    self.logger = self.latitude_coord
                    self.save = True
                self.setpoint[2] = self.longitude_coord + 8/105292.0089353767
                self.setpoint[1] = self.logger
                self.setpoint[0] = 24.16

            else:                                                 # Once the obstacle is out of sight of the Edrone
                self.setpoint[0] = 24.16                          # We apply the setpoints From the QR_Scanner
                self.setpoint[1] = self.qr_data[0]
                self.setpoint[2] = self.qr_data[1]
                self.setpoint4 = 1

        if self.setpoint4 == 1 and abs(self.lasterror[1]) < 0.000005 and abs(self.lasterror[2]) < 0.000005 and self.qr_data[0] -5/110692.0702932625 < self.latitude_coord < self.qr_data[0] + 5/110692.0702932625 and self.qr_data[1] - 5/105292.0089353767 < self.longitude_coord < self.qr_data[1] + 5/105292.0089353767:

            self.setpoint[0] = 4 + self.qr_data[2]
            self.setpoint3 = 0
            self.setpoint5 = 1
            self.setpoint2 = 0

        if self.setpoint5 == 1 and abs(self.lasterror[1]) < 0.000005 and abs(self.lasterror[2]) < 0.000005 and 4 + self.qr_data[2] < self.altitude_coord < 4.2 + self.qr_data[2] and self.err[0] < 0.5:

            self.Gripper_client(False)
            self.k_p = [550, 5750, 5750]
            self.k_i = [0.032, 560, 560]
            self.k_d = [222, 38000, 38000]
            self.setpoint[0] = 2 + self.qr_data[2]
            self.setpoint[1] = self.qr_data[0]
            self.setpoint[2] = self.qr_data[1]

        # Publishing msg drone_cmd
        self.drone_pub.publish(self.drone_cmd)

if __name__ == '__main__':
    time.sleep(1.5)
    E_DRONE = Edrone()
    R = rospy.Rate(16.67)
    while not rospy.is_shutdown():
        E_DRONE.pid()
        R.sleep()
