#!/usr/bin/python
'''Python script for Task4 '''

# Importing the required libraries

''' Python file for running Postion Controller '''

import csv
import time
from time import sleep
from sensor_msgs.msg import LaserScan
from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest
from std_msgs.msg import String
import rospy


class Edrone:

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
        self.x_error = 0.3
        self.y_error = 0.3
        self.building_num = 3
        self.cen_x = 0
        self.cen_y = 0
        self.detecting_mark = 0
        self.focal_length = 238.350718853
        self.saver = [0.0, 0.0, 0.0]
        self.bottom1 = [0]
        self.bottom = 0
        self.save = False
        self.save1 = False
        self.save2 = False
        self.save3 = False
        self.down = 0
        self.down1 = 0
        self.down2 = 0
        self.fwd = False
        self.bck = False
        self.left = False
        self.right = False
        self.clear = 0

        # Initalinzing Altitude ,Latitude and Longitude Error
        # Index 0 for Altitude , Index 1 for Latitude , Index 2 for Longitude error

        self.err = [0.0, 0, 0]
        self.allranges = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Seting Setpoint 1 Coordinates

        self.setpoint = [8.44099749139, 0.0, 71.9999430161]

        # Setting Kp, Ki, Kd values for altitude, latitude, longitude with mentioned indexing order

        self.k_p = [680, 6790, 6790]
        self.k_i = [0.032, 560, 560]
        self.k_d = [435, 42700, 42700]

        # Initializing Error Values for altitude, latitude, longitude with mentioned indexing order

        self.setpoint_recieved = [0, 0, 0, 0]
        self.lasterror = [0, 0, 0]
        self.errorsum = [0, 0, 0]
        self.output = [0, 0, 0]
        self.changerror = [0, 0, 0]

        # Declaring edrone_cmd and initialing Values

        self.setpoint_number = SetPoint_Number()
        self.setpoint_number.no = 0
        self.setpoint_numbers = 0
        self.drone_cmd = edrone_cmd()
        self.drone_cmd.rcThrottle = 0.0
        self.drone_cmd.rcRoll = 0.0
        self.drone_cmd.rcPitch = 0.0
        self.drone_cmd.rcYaw = 0.0
        self.setpoint_one_reached = 0
        self.sample_time = 0.06
        self.initial = 0
        self.my_setpoints = [[0, 0, 0]]

        # Declaring Publisher to topic drone_command for input to node attitude_controller.py

        self.drone_pub = rospy.Publisher('/drone_command', edrone_cmd,
                queue_size=1)
        self.setpoint_publisher = rospy.Publisher('/setpoint_number',
                SetPoint_Number, queue_size=1)

        # Subscribing to /edrone/gps to get cuurent altitude, latitude, longitude coordinates

        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps)
        rospy.Subscriber('/setpoint_list', setpoint_lists,
                         self.setpoint_reciever)
        rospy.Subscriber('/edrone/gripper_check', String, self.check)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan,
                         self.rangefinder)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan,
                         self.rangebottom)
        rospy.Subscriber('/center_pixels', pixel, self.pixel_reciever)

        with open('/home/tejasps28/catkin_ws/src/vitarana_drone/scripts/manifest.csv'
                  , 'r') as file:
            reader = csv.reader(file, delimiter=',')
            self.data = list(reader)

        # We read the msg and get the cuurent coordinates of the Edrone

    def pixel_reciever(self, msg):
        """ To revieve center pixel x and y"""

        self.cen_x = msg.x_cen_pixel
        self.cen_y = msg.y_cen_pixel
        self.detecting_mark = msg.detecting

    def gps(self, msg):
        """ gps runs each time coordinates are published to topic /edrone/gps """

        self.latitude_coord = msg.latitude
        self.longitude_coord = msg.longitude
        self.altitude_coord = msg.altitude

    def rangebottom(self, msg):
        """Bottom Range"""

        self.bottom1 = msg.ranges
        self.bottom = self.bottom1[0]

    def rangefinder(self, msg):
        """ To Get all the ranges from range_finder_top """

        self.allranges = msg.ranges

    def check(self, msg):
        """ To check if gripper can be activated """

        self.gripper = msg.data

    def Gripper_client(self, msg):
        """ Gripper Client Handler"""

        rospy.wait_for_service('/edrone/activate_gripper')
        try:
            service = rospy.ServiceProxy('/edrone/activate_gripper',
                    Gripper)
            resp = service(msg)
            return resp.result
        except rospy.ServiceException, e:

                                                                                                              # ##   35.2653964502

            print 'Service call failed: %s' % e

    def qr_receiver(self, msg):
        """ QR code Reciever """

        self.qr_data = map(float, msg.detected.split(','))

    def setpoint_reciever(self, msg):
        """ Recieves the Setpoint from List """

        self.setpoint_recieved = msg.list

    def setpoint_setter(self, number):
        self.my_setpoints = [
            [5, 19, 72, 0],
            [10.44099749139, 18.9999864489, 71.9999430161, 0],
            [8.45099749139, 18.9999864489, 71.9999430161, 1],
            [float(self.data[0][3]) + 8, 18.9999864489, 71.9999430161,
             0],
            [float(self.data[0][3]) + 8, float(self.data[0][1]),
             float(self.data[0][2]), 2],
            [float(self.data[0][3]) + 8, self.latitude_coord,
             self.longitude_coord, 0],
            [float(self.data[0][3]) + 8, self.latitude_coord,
             self.longitude_coord, 5],
            [11.44099749139, self.latitude_coord, self.longitude_coord,
             0],
            [13.44099749139, 19, 71.9999573, 0],
            [10.84099749139, 19, 71.9999573, 0],
            [8.45599749139, 19, 71.9999573, 1],
            [float(self.data[2][3]) + 8, 19, 71.9999573, 0],
            [float(self.data[2][3]) + 8, float(self.data[2][1]),
             float(self.data[2][2]), 4],
            [float(self.data[2][3]) + 8, float(self.data[2][1]),
             float(self.data[2][2]), 2],
            [float(self.data[2][3]) + 8, self.latitude_coord,
             self.longitude_coord, 0],
            [float(self.data[2][3]) + 8, self.latitude_coord,
             self.longitude_coord, 0],
            [float(self.data[2][3]) + 8 + 0.4, self.latitude_coord,
             self.longitude_coord, 5],
            [float(self.data[2][3]) + 4, self.latitude_coord,
             self.longitude_coord, 0],
            [float(self.data[2][3]) + 5, 19.0000135511, 71.9999430161,
             4],
            [float(self.data[2][3]) + 5, 19.0000135511, 71.9999430161,
             0],
            [11.841, 19.0000135511, 71.9999430161, 0],
            [8.45599749139, 19.0000135511, 71.9999430161, 1],
            [float(self.data[1][3]) + 8, 19.0000135511, 71.9999430161, 0],
            [float(self.data[1][3]) + 8, float(self.data[1][1]),
             float(self.data[1][2]), 2],
            [float(self.data[1][3]) + 8.02, self.latitude_coord,
             self.longitude_coord, 0],
            [float(self.data[1][3]) + 8 + 0.4, self.latitude_coord,
             self.longitude_coord, 5],
            [float(self.data[1][3]) + 4, self.latitude_coord,
             self.longitude_coord, 0],
            [float(self.data[1][3]) + 4, self.saver[1], self.saver[2], 0],
            [self.saver[0], self.saver[1], self.saver[2], 0]
            ]

        if number <= len(self.my_setpoints):
            return self.my_setpoints[number]
        else:
            return self.my_setpoints[len(self.my_setpoints)]

    def pid(self):
        """  Main PID Controller for altitude, latitude, longitude Error """

        def range_check(self, arglist):
            """ To check the ranges and return True if range < 17 """

            if type(arglist[0]) == float:
                if 1 < arglist[0] < 18.0:
                    self.longitude_right_range = True
                else:
                    self.longitude_right_range = False

            if type(arglist[1]) == float:
                if 1 < arglist[1] < 18.0:
                    self.latitude_back_range = True
                else:
                    self.latitude_back_range = False

            if type(arglist[2]) == float:
                if 1 < arglist[2] < 18.0:
                    self.longitude_left_range = True
                else:
                    self.longitude_left_range = False

            if type(arglist[3]) == float:
                if 1 < arglist[3] < 18.0:
                    self.latitude_fwd_range = True
                else:
                    self.latitude_fwd_range = False
            return [self.longitude_right_range,
                    self.latitude_back_range,
                    self.longitude_left_range, self.latitude_fwd_range]

        # Calculating Error for altitude, latitude, longitude

        self.err[0] = self.setpoint[0] - self.altitude_coord
        self.err[1] = self.setpoint[1] - self.latitude_coord
        self.err[2] = self.setpoint[2] - self.longitude_coord

        # Calculating Change in Error for altitude, latitude, longitude

        self.changerror[0] = self.err[0] - self.lasterror[0]
        self.changerror[1] = self.err[1] - self.lasterror[1]
        self.changerror[2] = self.err[2] - self.lasterror[2]

        # Calculating sum of Error for altitude, latitude, longitude

        self.errorsum[0] = (self.errorsum[0] + self.err[0]) \
            * self.sample_time
        self.errorsum[1] = (self.errorsum[1] + self.err[1]) \
            * self.sample_time
        self.errorsum[2] = (self.errorsum[2] + self.err[2]) \
            * self.sample_time

        # Calculating Output which is to be sent to attitude_controller.py through edrone/cmd pub

        self.output[0] = self.k_p[0] * self.err[0] + self.k_i[0] \
            * self.errorsum[0] + self.k_d[0] * self.changerror[0] \
            / self.sample_time
        self.output[1] = self.k_p[1] * self.err[1] + self.k_i[1] \
            * self.errorsum[1] + self.k_d[1] * self.changerror[1] \
            / self.sample_time
        self.output[2] = self.k_p[2] * self.err[2] + self.k_i[2] \
            * self.errorsum[2] + self.k_d[2] * self.changerror[2] \
            / self.sample_time

        # Equation for Throttle , Pitch, Roll and Yaw values for attitude_controller.py

        self.drone_cmd = edrone_cmd()
        self.drone_cmd.rcThrottle = 1.2 * self.output[0] + 1500
        self.drone_cmd.rcPitch = 1500 + 5.1 * self.output[1]
        self.drone_cmd.rcRoll = 1500 + 5.1 * self.output[2]
        self.drone_cmd.rcYaw = 1500

        # Storing Current error

        self.lasterror[0] = self.err[0]
        self.lasterror[1] = self.err[1]
        self.lasterror[2] = self.err[2]

        self.x_m = self.cen_x / self.focal_length
        self.y_m = self.cen_y / self.focal_length

        # Check to see If we have reached Setpoint 1 within allowed error range

        if self.setpoint[0] - 0.1 < self.altitude_coord \
            < self.setpoint[0] + 0.1 and self.setpoint[1] - 0.000003517 \
            < self.latitude_coord < self.setpoint[1] + 0.000003517 \
            and self.setpoint[2] - 0.00000749 < self.longitude_coord \
            < self.setpoint[2] + 0.000003749 \
            and self.setpoint_setter(self.setpoint_numbers)[3] == 0:
            self.setpoint_numbers = self.setpoint_numbers + 1
            self.setpoint = \
                self.setpoint_setter(self.setpoint_numbers)[0:3]
            print 'haha' * 2
        elif self.setpoint_setter(self.setpoint_numbers)[3] == 1 \
            and abs(self.lasterror[1]) < 0.0000004 \
            and abs(self.lasterror[2]) < 0.0000004:

            if self.gripper == self.str:
                self.bool = True
                if self.Gripper_client(self.bool):  # If We get a "True" Response from Gripper then we can proceed
                    self.k_p[1] = 1600
                    self.k_p[2] = 1750
                    self.k_i[1] = 3437
                    self.k_i[2] = 3437
                    self.k_d[1] = 23870
                    self.k_d[2] = 23870
                    self.setpoint_numbers = self.setpoint_numbers + 1
                    self.setpoint = \
                        self.setpoint_setter(self.setpoint_numbers)[0:3]
        elif self.setpoint_setter(self.setpoint_numbers)[3] == 2 \
            and abs(self.lasterror[1]) < 0.000085 \
            and abs(self.lasterror[2]) < 0.000085 \
            and self.detecting_mark == 1:

            self.setpoint[1] = self.latitude_coord + self.x_m \
                * (self.altitude_coord - 8) / 110692.0702932625
            self.setpoint[2] = self.longitude_coord + self.y_m \
                * (self.altitude_coord - 8) / 105292.0089353767
            if self.x_error < 0.140 and self.y_error < 0.140 \
                and abs(self.lasterror[1]) < 0.00000058 \
                and abs(self.lasterror[2]) < 0.00000058 \
                and abs(self.changerror[1]) < 0.00000058 \
                and abs(self.changerror[2]) < 0.00000058:
                self.setpoint_numbers = self.setpoint_numbers + 1
                self.setpoint = \
                    self.setpoint_setter(self.setpoint_numbers)[0:3]
        elif self.setpoint_setter(self.setpoint_numbers)[3] == 5:

            self.setpoint[0] = self.setpoint[0] - 0.2
            print 'going down' * 50
            if self.setpoint_setter(self.setpoint_numbers)[0] - 8 \
                - 0.59 < self.altitude_coord \
                < self.setpoint_setter(self.setpoint_numbers)[0] - 8 \
                + 0.59 and self.setpoint[1] - 0.000003517 \
                < self.latitude_coord < self.setpoint[1] + 0.000003517 \
                and self.setpoint[2] - 0.000003749 \
                < self.longitude_coord < self.setpoint[2] + 0.000003749:
                print 'yay' * 50
                self.Gripper_client(False)
                print self.Gripper_client(False)
                self.k_p = [680, 6790, 6790]
                self.k_i = [0.032, 560, 560]
                self.k_d = [435, 42700, 42700]
                self.setpoint_numbers = self.setpoint_numbers + 1
                self.setpoint = \
                    self.setpoint_setter(self.setpoint_numbers)[0:3]


        elif self.setpoint_setter(self.setpoint_numbers)[3] == 4:

            if range_check(self, self.allranges)[3] \
                and self.setpoint_setter(self.setpoint_numbers)[1] \
                < self.latitude_coord:  # If eDrone in near on obstacle from the front then we get True resp
                print 'ahead' * 50
                if not self.save:  # Hence we avoid the obstacle by changing setpoint
                    self.logger = self.latitude_coord
                    self.save = True
                self.setpoint[2] = self.longitude_coord + 17 \
                    / 105292.0089353767
                self.setpoint[1] = self.logger
            elif range_check(self, self.allranges)[1] \
                and self.latitude_coord \
                < self.setpoint_setter(self.setpoint_numbers)[1]:

                if not self.save1:  # Hence we avoid the obstacle by changing setpoint
                    self.logger = self.latitude_coord
                    self.save1 = True
                self.setpoint[2] = self.longitude_coord + 17 \
                    / 105292.0089353767
                self.setpoint[1] = self.logger
            elif range_check(self, self.allranges)[2] \
                and self.setpoint_setter(self.setpoint_numbers)[3] \
                < self.longitude_coord:

                if not self.save2:
                    self.logger = self.longitude_coord
                    self.save2 = True
                self.setpoint[1] = self.latitude_coord + 10 \
                    / 110692.0702932625
                self.setpoint[2] = self.logger
            elif range_check(self, self.allranges)[0] \
                and self.longitude_coord \
                < self.setpoint_setter(self.setpoint_numbers)[3]:

                if not self.save3:
                    self.logger = self.longitude_coord
                    self.save3 = True
                self.setpoint[1] = self.latitude_coord - 10 \
                    / 110692.0702932625
                self.setpoint[2] = self.logger
            elif self.setpoint[0] - 0.1 < self.altitude_coord \
                < self.setpoint[0] + 0.1 \
                and self.setpoint_setter(self.setpoint_numbers)[2] \
                - 0.00007849 < self.longitude_coord \
                < self.setpoint_setter(self.setpoint_numbers)[2] \
                + 0.00007849 \
                and self.setpoint_setter(self.setpoint_numbers)[1] \
                - 0.000065517 < self.latitude_coord \
                < self.setpoint_setter(self.setpoint_numbers)[1] \
                + 0.000065517:

                print 'changing setpoint' * 50
                self.setpoint_numbers = self.setpoint_numbers + 1
                self.setpoint = \
                    self.setpoint_setter(self.setpoint_numbers)[0:3]
            else:
                self.setpoint = \
                    self.setpoint_setter(self.setpoint_numbers)[0:3]
                self.fwd = False
                self.bck = False
                self.left = False
                self.right = False

        if self.initial == 0 and self.setpoint[1] == 0.0:
            print "Saved"*50
            self.saver = [self.altitude_coord, self.latitude_coord,
                          self.longitude_coord]
            self.saver = [self.altitude_coord, self.latitude_coord,
                          self.longitude_coord]
            print "saveddddddd"*50

            print 'fatooo'*50

        if self.saver[0] != 0.0 and self.clear == 0:
                self.setpoint[0] = 10.44099
                self.setpoint[1] = 19
                self.setpoint[2] = 72
                self.initial = 1
                self.clear = 1
                print 'here'

        self.x_error = self.cen_x * (self.altitude_coord - 10.7) \
            / self.focal_length
        self.y_error = self.cen_y * (self.altitude_coord - 10.7) \
            / self.focal_length

        self.drone_pub.publish(self.drone_cmd)

        print self.saver
        print self.initial


if __name__ == '__main__':
    time.sleep(2.5)
    E_DRONE = Edrone()
    R = rospy.Rate(16.67)
    while not rospy.is_shutdown():
        E_DRONE.pid()
        R.sleep()
