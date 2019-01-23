#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class WebcamCapture():
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.start_pub()

    def start_pub(self):
        bridge = CvBridge()
        pub = rospy.Publisher('/cam_feed', Image, queue_size=25)
        rospy.init_node('webcam', anonymous=True)
        rate = rospy.Rate(12)

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            rospy.loginfo("Frame captured from webcam")
            if not frame.empty():
                cv2.imshow('Webcam', frame)
                pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
                rospy.loginfo("Image message sent; message type: %s", type(frame))
            rate.sleep()
            if cv2.waitKey(1) == 27:
                break
        self.cap.release()
        cv2.destroyAllWindows()


def main():
    WebcamCapture()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
