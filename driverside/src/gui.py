#!/usr/bin/env python

import sys
import rospy
import cv2
import PySide2
from PySide2.QtWidgets import QWidget
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class GUI(QWidget):
    def __init__(self):
        super(GUI, self).__init__()

        self.initUI()

    def initUI(self):
        # setup menubar
        menubar = self.menuBar()

        # setup file menu and all its items
        file_menu = menubar.addMenu('&File')

        # setup debug menu and all its items
        debug_menu = menubar.addMenu('&Debug')

        # setup controls menu and all its items
        controls_menu = menubar.addMenu('&Controls')

        # setup status bar
        self.statusBar()

        # setup Video object for camera feeds
        vid = Video()

        self.setWindowTitle('ASU MATE 2019 Video Feed')
        self.showMaximized()


class Video(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self._bridge = CvBridge()

        self._frame = None
        self._img = None

        self._label = QLabel(self)
        # self._pxmp = QPixmap()
        # self._pxmp.fill(QColor.setNamedColor('black'))
        self._label.setPixmap(QPixmap.fill(QColor.setNamedColor('black')))

        self._layout = QVBoxLayout()
        self._layout.addWidget(self._label)
        self.setLayout(self._layout)

    def vid_callback(self, data):
        try:
            # convert received ROS Image to cv2 image
            self._frame = self._bridge.imgmsg_to_cv2(data, "bgr8")
            print("Frame received")

            # convert cv2 image to QPixmap
            self._img = QImage(self._frame, self._frame.shape[1], self_frame.shape[0])
            print("Frame converted")

            # paint QPixmap as label
            self._label.setPixmap(QPixmap.fromImage(self._img))
            print("Frame painted")

            self.update()
        # rewrite in Python 2
        except CvBridgeError as e:
            print(e)


def main():
    app = QApplication([])

    # create node
    rospy.init_node('gui')
    gui = GUI()

    rospy.Subscriber("cam_front", Image, gui.prim_vid.vid_callback())
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
