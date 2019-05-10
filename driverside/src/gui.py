#!/usr/bin/env python

import sys
import rospy
import cv2
from PySide2.QtWidgets import QWidget, QApplication, QMenuBar, QLabel, QVBoxLayout
from PySide2.QtGui import QPixmap, QColor, QImage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class GUI(QWidget):
    def __init__(self):
        super(GUI, self).__init__()

        self.init_ui()

    def init_ui(self):
        self.vid = Video()

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.vid)
        self.setLayout(self.layout)

        self.setWindowTitle('Video Feed')
        self.showMaximized()


class Video(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self._bridge = CvBridge()

        self._frame = None
        self._img = None

        self._label = QLabel(self)
        self._pxmp = QPixmap()
        self._pxmp.fill(QColor('black'))
        self._label.setPixmap(self._pxmp)

        self._layout = QVBoxLayout()
        self._layout.addWidget(self._label)
        self.setLayout(self._layout)

    def vid_callback(self, data):
        try:
            # convert received ROS Image to cv2 image and convert to RGB
            self._frame = cv2.cvtColor(self._bridge.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)

            # convert cv2 image to QPixmap
            self._img = QImage(self._frame.data, self._frame.shape[1], self._frame.shape[0], self._frame.strides[0],
                               QImage.Format_RGB888)

            # set self._label to pixmap
            self._label.setPixmap(QPixmap.fromImage(self._img))

            self.update()

        except CvBridgeError, e:
            print e


def main():
    app = QApplication([])

    # create node
    rospy.init_node('VidSub')
    gui = GUI()

    # define subscribed topics, data types, and callback functions
    rospy.Subscriber('proc_vid_feed', Image, gui.vid.vid_callback)

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
