#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def main():
    bridge = CvBridge()
    pub = rospy.Publisher('vid_feed', Image, queue_size=25)
    rospy.init_node('VidPub', anonymous=True)
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        pub.publish(bridge.cv2_to_imgmsg(frame))
        if cv2.waitKey(1) == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
