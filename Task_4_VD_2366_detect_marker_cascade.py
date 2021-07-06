#!/usr/bin/env python

''' This is a Script For Detecting Marker Using the Edrone Camera '''
from sensor_msgs.msg import Image
from vitarana_drone.msg import *
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy  
import cv2
from matplotlib import pyplot as plt


class DetectMarker():
    ''' Class Object For Marker Detection '''
    def __init__(self):
        ''' Initializing the Node '''
        rospy.init_node('marker_detector_node') # Initializing Rosnode
        self.logo_cascade = cv2.CascadeClassifier('/home/tejasps28/Desktop/intro_cascade_classifier_training_and_usage/data/cascade.xml')
        self.camera_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.camera_callback)
        self.img = np.empty([1,1,3])
        self.bridge = CvBridge()
        self.center = pixel()
        self.center.x_cen_pixel = 0
        self.center.y_cen_pixel = 0
        self.center.detecting = 0
        self.saved_x = 0
        self.saved_y = 0

        self.pixels_pub = rospy.Publisher('/center_pixels', pixel,queue_size=1)



    def camera_callback(self,camera):
        ''' Callback function for camera '''
        try:
            self.img = self.bridge.imgmsg_to_cv2(camera, "bgr8") # Converting to OpenCV standard img
            cv2.waitKey(5)
        except CvBridgeError as err:
            print err
            return
    def detector(self):
        ''' Main Function for detecting the marker '''
        height, width = self.img.shape[:2]
        cv2.waitKey(5)
        copyimg = np.uint8(self.img)
        gray = cv2.cvtColor(copyimg, cv2.COLOR_BGR2GRAY)
       #  image, reject levels level weights.
        logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05)
        for (x, y, w, h) in logo:
            cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
            cv2.circle(self.img , ((2*x + w )/2 , (2*y + h)/2) , 1,(255, 0, 0), 2)
            print (2*x + w)/2
            print (2*y + h)/2
            self.center.x_cen_pixel = (2*x + w)/2 - 200
            self.center.y_cen_pixel = (2*y + h)/2 - 200
        if self.center.x_cen_pixel == self.saved_x and self.center.y_cen_pixel == self.saved_y:
            self.center.detecting = 0
        else:
            self.center.detecting = 1
        self.saved_x = self.center.x_cen_pixel
        self.saved_y = self.center.y_cen_pixel
        plt.ion()
        plt.pause(0.0001)
        plt.imshow(cv2.cvtColor(copyimg, cv2.COLOR_BGR2RGB))
        plt.show()
        self.pixels_pub.publish(self.center)

if __name__ == '__main__':
    DETECT_MARKER = DetectMarker()
    R = rospy.Rate(1)
    while not rospy.is_shutdown():
        DETECT_MARKER.detector()
        R.sleep()
