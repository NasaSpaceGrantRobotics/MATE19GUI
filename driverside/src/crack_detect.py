#!/usr/bin/env python

import math
import numpy as np
import cv2
import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from sensor_msgs.msg import Image


class CrackDetect:
    def __init__(self):
        self.bridge = CvBridge()
        self.l_sub = rospy.Subscriber('zed/rgb/image_raw_color', Image)
        self.r_sub = rospy.Subscriber('zed/right/image_raw_color', Image)
        self.l_frame = None
        self.r_frame = None
        self.pub = rospy.Publisher('crack_size', Float32, queue_size=25)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.l_sub, self.r_sub], 50)
        self.ts.registerCallback(self.callback)

    def callback(self, l_data, r_data):
        self.l_frame = cv2.cvtColor(self.bridge.imgmsg_to_cv2(l_data), cv2.COLOR_BGR2RGB)
        self.r_frame = cv2.cvtColor(self.bridge.imgmsg_to_cv2(r_data), cv2.COLOR_BGR2RGB)
        cv2.imshow('zed/rgb/image_raw_color', self.l_frame)
        cv2.imshow('zed/right/image_raw_color', self.r_frame)

        self.pub.publish(self.compute_crack_size)

    def bin_to_binding_rect(self, frame):
        # perform thresholding
        # perform rectangle binding
        #
        x, y, w, h = cv2.boundingRect(frame)  # countours[0]
        return cv2.rectangle(frame, )

        return 0.00


def main():
    rospy.init_node('CrackDetect')
    CrackDetect()
    rospy.spin()


if __name__ == '__main__':
    main()
