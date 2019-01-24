#!/usr/bin/env python

import sys
import rospy
import cv2
from PySide2.QtWidgets import QWidget, QApplication, QMenuBar, QStatusBar, QLabel, QVBoxLayout
from PySide2.QtGui import QPixmap, QColor, QImage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class GUI(QWidget):
    def __init__(self):
        super(GUI, self).__init__()

        self.initUI()

    def initUI(self):
        # setup menubar
        menu_bar = QMenuBar(self)

        # setup file menu and all its items
        file_menu = menu_bar.addMenu('&File')

        # setup debug menu and all its items
        debug_menu = menu_bar.addMenu('&Debug')

        # setup controls menu and all its items
        controls_menu = menu_bar.addMenu('&Controls')

        # setup status bar
        self.status_bar = QStatusBar(self)

        # setup Video object for camera feeds
        self.vid = Video()

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.vid)
        self.setLayout(self.layout)

        self.setWindowTitle('ASU MATE 2019 Video Feed')
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
            # convert received ROS Image to cv2 image
            self._frame = self._bridge.imgmsg_to_cv2(data, 'bgr8')
            rospy.loginfo("Frame received; message type: %s", type(self._frame))

            # convert cv2 image to QPixmap
            self._img = QImage(self._frame.data, self._frame.shape[1], self._frame.shape[0], self._frame.strides[0], QImage.Format_RGB888)
            rospy.loginfo("Frame converted: frame type: %s", type(self._img))

            # paint QPixmap as label
            self._label.setPixmap(QPixmap.fromImage(self._img))
            rospy.loginfo("Frame painted")

            self.update()
        # rewrite in Python 2
        except CvBridgeError, e:
            print e


def main():
    app = QApplication([])

    # create node
    rospy.init_node('vidSub')
    gui = GUI()

    # define subscribed topics, data types, and callback functions
    rospy.Subscriber('vidFeed', Image, gui.vid.vid_callback)

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
