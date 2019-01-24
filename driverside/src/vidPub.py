#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def main():
    bridge = CvBridge()
    pub = rospy.Publisher('vidFeed', Image, queue_size=25)
    rospy.init_node('vidPub', anonymous=True)
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        cv2.imshow('Webcam', frame)
        pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
        if cv2.waitKey(1) == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
