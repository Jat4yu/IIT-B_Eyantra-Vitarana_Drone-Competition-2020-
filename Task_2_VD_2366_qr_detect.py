#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''

from pyzbar.pyzbar import decode
from PIL import Image
from sensor_msgs.msg import Image as Imager
from cv_bridge import CvBridge, CvBridgeError
from vitarana_drone.msg import *
import cv2
import numpy as np
import rospy

class image_proc():

	# Initialise everything
    def __init__(self):
        rospy.init_node('barcode_test') #Initialise rosnode
        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Imager, self.image_callback) #Subscribing to the camera topic
        self.qr_pub = rospy.Publisher("/qr_info", QR_scanned, queue_size=1)
        self.img = np.empty([1, 1, 1]) # This will contain your image frame from camera
        self.bridge = CvBridge()
        self.decoded = [0, 0, 0]
        self.a = ["-1"]
        self.qr_detector = QR_scanned()
        self.qr_detector.detected = ''
	# Callback function of amera topic
    def image_callback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
            cv2.waitKey(5)
        except CvBridgeError as e:
            print(e)
            return
    def Decode(self):
        height, width = self.img.shape[:2]
        cv2.waitKey(5)
        self.decoded = decode((self.img[:, :, 0].astype('uint8').tobytes(), width, height))
        cv2.waitKey(5)
        for obj in self.decoded:
            self.a[0] = obj[0]
        self.qr_detector = QR_scanned()
        self.qr_detector.detected = self.a[0]
        self.qr_pub.publish(self.qr_detector)

if __name__ == '__main__':
    image_proc_obj = image_proc()
    R = rospy.Rate(16.67)
    while not rospy.is_shutdown():
        image_proc_obj.Decode()
        R.sleep()
