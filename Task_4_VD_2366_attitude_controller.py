#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control
which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /roll_error             /pid_tuning_altitude
        /pitch_error            /pid_tuning_pitch
        /yaw_error              /pid_tuning_roll
        /edrone/pwm             /edrone/imu/data
                                /edrone/drone_command

Rather than using different variables, use list.
eg : self.setpoint = [1,2,3], where index corresponds to x,y,z
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS
WHILE CODE EVALUATION.
'''

# Importing the required libraries
import time
from time import sleep
from vitarana_drone.msg import *
#from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import tf


class Edrone():
    """docstring for Edrone"""
    def __init__(self):

        rospy.init_node('attitude_controller')  # initializing ros node with name drone_control

        # This corresponds to your current orientation of eDrone in quaternion format.
        #This value must be updated each time in your imu callback
        # [x,y,z,w]

        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [r,p,y]

        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command
        ## in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]

        self.setpoint_cmd = [1500.0, 1500.0, 1500.0, 1000.0]

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]

        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring Error msg values For PlotJuggler

        ## self.roll_errormsg = roll_error()
        ## self.pitch_errormsg = pitch_error()
        ## self.yaw_errormsg = yaw_error()

        # Declaring pwm_cmd of message type prop_speed and initializing values

        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # initial setting of Kp, Kd and ki for [roll, pitch, yaw].
        # eg: self.Kp[2] corresponds to Kp value in yaw axis
        # after tuning these are computed corresponding PID parameters
        # Initializing Other parameters
        self.k_p = [240, 240, 900]
        self.k_i = [0, 0, 0]
        self.k_d = [78, 78, 8.6]
        self.lastime = 0
        self.error = [0, 0, 0]
        self.error_sum = [0, 0, 0]
        self.lasterror = [0, 0, 0]
        self.changerror = [0, 0, 0]
        self.output = [0, 0, 0]

        # This is the sample time in which you need to run pid. Choose any time which you seem fit.
        # Stimulation step time is 50 ms in seconds

        self.sample_time = 0.06

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error\

        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)

        ## self.roll_error = rospy.Publisher('/edrone/roll_error', roll_error, queue_size=1)
        ## self.pitch_error = rospy.Publisher('/edrone/pitch_error', pitch_error, queue_size=1)
        ## self.yaw_error = rospy.Publisher('/edrone/yaw_error', yaw_error, queue_size=1)



        #Subscribing to /drone_command ,imu/data

        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)

        #rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        #rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        #rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)

        # Note: The imu publishes various kind of data viz angular velocity
        # linear acceleration, magnetometer reading (if present),
        # But here we are interested in the orientation
        # which can be calculated by a complex algorithm called filtering.
        # which is not in the scope of this task,

    def imu_callback(self, msg):
        """ CallbackFunction For IMU"""
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w


    def drone_command_callback(self, msg):
        """ CallbackFunction For drone_command """
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.setpoint_cmd[3] = msg.rcThrottle

    def pid(self):
        """ Main Algorithm For PID Controller """


        # Converting quaternion to euler angles

        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for all axis

        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30  #Roll
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30  #Pitch
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30  #Yaw

        #Calculating Error, Change in Error and Sum of Error. For Kp ,Kd ,Ki respectively

        self.error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0] #Roll Error
        self.error[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1] #Pitch Error
        self.error[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2] #Yaw Error

        self.changerror[0] = self.error[0] - self.lasterror[0] #Change in error d(e)
        self.changerror[1] = self.error[1] - self.lasterror[1]
        self.changerror[2] = self.error[2] - self.lasterror[2]

        self.error_sum[0] = self.error_sum[0] + self.error[0]*self.sample_time #Sum of Errors
        self.error_sum[1] = self.error_sum[1] + self.error[1]*self.sample_time
        self.error_sum[2] = self.error_sum[2] + self.error[2]*self.sample_time

        # Calculating Output

        self.output[0] = self.k_p[0]*self.error[0] + self.k_i[0]*self.error_sum[0] + self.k_d[0]*self.changerror[0]/self.sample_time #Output to PWM For Roll
        self.output[1] = self.k_p[1]*self.error[1] + self.k_i[1]*self.error_sum[1] + self.k_d[1]*self.changerror[1]/self.sample_time #Output to PWM For Pitch
        self.output[2] = self.k_p[2]*self.error[2] + self.k_i[2]*self.error_sum[2] + self.k_d[2]*self.changerror[2]/self.sample_time #Output to PWM For Yaw


        # Converting the range of 1000 to 2000 to 0 to 1024 for throttle here itself

        pwmthrottleout = 1.024*self.setpoint_cmd[3] - 1024

        self.pwm_cmd = prop_speed()

        # Propeller Equation for individual propellers , Controllong Roll Pitch Yaw independently of one another

        self.pwm_cmd.prop1 = max(min(pwmthrottleout + self.output[0] - self.output[1] - self.output[2], 1023), 0)
        self.pwm_cmd.prop2 = max(min(pwmthrottleout - self.output[0] - self.output[1] + self.output[2], 1023), 0)
        self.pwm_cmd.prop3 = max(min(pwmthrottleout - self.output[0] + self.output[1] - self.output[2], 1023), 0)
        self.pwm_cmd.prop4 = max(min(pwmthrottleout + self.output[0] + self.output[1] + self.output[2], 1023), 0)

        self.lasterror[0] = self.error[0] # Storing Current Errors for the next loop
        self.lasterror[1] = self.error[1]
        self.lasterror[2] = self.error[2]

        # Publishing Propeller Command
        
        self.pwm_pub.publish(self.pwm_cmd)

        ##self.roll_error.publish(self.error[0])   Commented out for PID Tuning Please Ignore
        ##self.pitch_error.publish(self.error[1])
        ##self.yaw_error.publish(self.error[2])


if __name__ == '__main__':
    time.sleep(2)
    E_DRONE = Edrone()
    R = rospy.Rate(16.67)  # specify rate in Hz based upon your desired PID sampling time
    while not rospy.is_shutdown():
        E_DRONE.pid()
        R.sleep()
