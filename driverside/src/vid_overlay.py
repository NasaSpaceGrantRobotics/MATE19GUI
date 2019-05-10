#!/usr/bin/env python

import math
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
# from std_msgs.msg import Twist


class VideoOverlay:
    def __init__(self):
        self._bridge = CvBridge()
        self._pub = rospy.Publisher('proc_vid_feed', Image, queue_size=25)

        self._frame = None
        self._north_offset = math.pi / 2
        self._depth = 0
        self._ph = 7
        self._induction = False

    def grab_direction(self, data):
        self._north_offset = data.data

    def grab_depth(self, data):
        self._depth = data

    def grab_ph(self, data):
        self._ph = data

    def grab_induction(self, data):
        self._induction = data

    def draw_overlay(self, data):
        try:
            self._frame = self._bridge.imgmsg_to_cv2(data)
            self.put_data(self._frame)
            self.draw_compass(self._frame, self._north_offset)
            self._pub.publish(self._bridge.cv2_to_imgmsg(self._frame))

        except Exception, e:
            print e

    def put_data(self, frame):
        depth = 'Depth: ' + str(self._depth)
        ph = 'Ph: ' + str(self._ph)
        induction = 'Induction: ' + str(self._induction)

        cv2.putText(frame, depth, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, ph, (5, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, induction, (5, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

    def draw_compass(self, frame, offset):
        R1 = frame.shape[0] / 2 - 10  # radius of outer tick marks
        r1 = frame.shape[0] / 24  # length of tick marks
        R2 = R1 - r1 - 10  # radius of inner tick marks
        R3 = R2 - r1 - 10  # radius of labels

        width = frame.shape[1]
        height = frame.shape[0]

        # draw center crosshair
        cv2.line(frame, ((width / 2) - (height / 8), height / 2), ((width / 2) + (height / 8), height / 2),
                 (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(frame, (width / 2, (height / 2) - (height / 8)), (width / 2, (height / 2) + (height / 8)),
                 (255, 255, 255), 1, cv2.LINE_AA)

        # compute angles of each cardinal direction in relation to offset
        n_angle = math.pi/2 - offset
        w_angle = math.pi - offset
        s_angle = (3*math.pi)/2 - offset
        e_angle = -offset

        # compute coordinates for where N, S, E, and W will lie along outlining circle
        n = (int(width/2 + R3*math.cos(n_angle)), int(height/2 - R3*math.sin(n_angle)))
        w = (int(width/2 + R3*math.cos(w_angle)), int(height/2 - R3*math.sin(w_angle)))
        s = (int(width/2 + R3*math.cos(s_angle)), int(height/2 - R3*math.sin(s_angle)))
        e = (int(width/2 + R3*math.cos(e_angle)), int(height/2 - R3*math.sin(e_angle)))

        # draw outer tick marks
        for i in range(0, 360, 5):
            if i % 6 == 0 and i != 270:
                cv2.line(frame, (int(width / 2 + R1 * math.cos(i * (math.pi / 180))),
                                 int(height / 2 + R1 * math.sin(i * (math.pi / 180)))), (
                         int(width / 2 + R1 * math.cos(i * (math.pi / 180)) - r1 * math.cos(i * (math.pi / 180))),
                         int(height / 2 + R1 * math.sin(i * (math.pi / 180)) - r1 * math.sin(i * (math.pi / 180)))),
                         (255, 255, 255), 2, cv2.LINE_AA)
            else:
                cv2.line(frame, (int(width/2 + R1*math.cos(i*(math.pi/180))),
                                 int(height/2 + R1*math.sin(i*(math.pi/180)))),
                         (int(width/2 + R1*math.cos(i*(math.pi/180)) - r1*math.cos(i*(math.pi/180))),
                          int(height/2 + R1*math.sin(i*(math.pi/180)) - r1*math.sin(i*(math.pi/180)))),
                         (255, 255, 255), 1, cv2.LINE_AA)

            if i % 15 == 0:
                cv2.line(frame, (int(width/2 + R2 * math.cos(i * (math.pi/180) - offset)), int(height/2 + R2 * math.sin(i * (math.pi/180) - offset))),
                         (int(width/2 + R2 * math.cos(i * (math.pi/180) - offset) - r1 * math.cos(i * (math.pi/180) - offset)),
                         int(height/2 + R2 * math.sin(i * (math.pi/180) - offset) - r1 * math.sin(i * (math.pi/180) - offset))),
                         (255, 255, 255), 1, cv2.LINE_AA)

        # draw arrowtip in direction of robot
        self.draw_arrowtip(frame, (width/2, r1 + 10), (2 * r1) / math.sqrt(3), math.pi / 2, (255, 255, 255))

        # draw inner tick marks
        # for i in range(0, 360, 15):
        #     cv2.line(frame, (int(width / 2 + R2 * math.cos(i * (math.pi / 180) - offset)), int(height / 2 + R2 * math.sin(i * (math.pi / 180) - offset))),
        #              (int(width / 2 + R2 * math.cos(i * (math.pi / 180) - offset) - r1 * math.cos(i * (math.pi / 180) - offset)),
        #              int(height / 2 + R2 * math.sin(i * (math.pi / 180) - offset) - r1 * math.sin(i * (math.pi / 180) - offset))),
        #              (255, 255, 255), 1, cv2.LINE_AA)

        # draw arrowtip in direction of North
        self.draw_arrowtip(frame, (int(width / 2 + R2 * math.cos(math.pi / 2 - offset)),
                                   int(height / 2 - R2 * math.sin(math.pi / 2 - offset))),
                           (2 * r1) / math.sqrt(3), (3 * math.pi)/2 - offset, (0, 0, 255))

        # overlay N, W, S, E labels on compass
        self.put_tangent_text(frame, 'N', n, n_angle, (0, 0, 255))
        self.put_tangent_text(frame, 'W', w, w_angle, (255, 255, 255))
        self.put_tangent_text(frame, 'S', s, s_angle, (255, 255, 255))
        self.put_tangent_text(frame, 'E', e, e_angle, (255, 255, 255))

    def draw_arrowtip(self, frame, pt, length, angle, color):
        pts = np.array([[[pt[0], pt[1]],
                         [pt[0] + length * math.cos(angle - math.pi/6), pt[1] - length * math.sin(angle - math.pi/6)],
                         [pt[0] + length * math.cos(angle + math.pi/6), pt[1] - length * math.sin(angle + math.pi/6)]]],
                       dtype=np.int32)
        cv2.fillPoly(frame, pts, color, cv2.LINE_AA)

    def put_tangent_text(self, frame, text, pt, angle, color):
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
        r = math.sqrt((text_size[0])[0] ** 2 + (text_size[0])[1] ** 2) / 2
        x = int(pt[0] - r * math.cos(angle) + r * math.cos((5 * math.pi)/4))
        y = int(pt[1] + r * math.sin(angle) - r * math.sin((5 * math.pi)/4))
        cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)


def main():
    overlay = VideoOverlay()
    rospy.init_node('OverlayDraw')
    rospy.Subscriber('camera', Image, overlay.draw_overlay)
    rospy.Subscriber('compass_offset', Float32, overlay.grab_direction)
    # rospy.Subscriber('imu_data', Twist, overlay.grab_imu_values)
    rospy.spin()


if __name__ == '__main__':
    main()
