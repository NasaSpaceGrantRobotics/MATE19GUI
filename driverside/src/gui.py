#!/usr/bin/env python

import sys
# import rospy
import cv2
from import PySide2.QtWidgets import *
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
        """
        restart_item = QAction('&Restart')
        restart_item.setStatusTip('Relaunch Application')
        restart_item.triggered.connect(self.restart())


        quit_item = QAction('&Quit', self)
        quit_item.setStatusTip('Quit Application')
        quit_item.triggered.connect(qApp.quit)
        file_menu.addAction(quit_item)
        """
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
        self._image = None
        self._label = QLabel(self)
        self._pxmp = QPixmap()
        self._pxmp.fill(QColor(black)
        self._label.setPixmap(self._image)

    def vid_callback(self, data):
        try:
            # convert received ROS Image to cv2 image
            self._frame = self._bridge.imgmsg_to_cv2(data, "bgr8")
            print("Frame received")

            # convert cv2 image to QPixmap
            self._image = QImage(self._frame.data)
            self._pxmp = QPixmap.fromImage(self._image)
            print("Frame converted")

            # paint QPixmap as label
            self._label.setPixmap(self._pxmp)
            print("Frame painted")

            self.update()
        # rewrite in Python 2
        except CvBridgeError as e:
            print(e)


def main():
    app = QApplication([])

    # create node
    # rospy.init_node('gui')
    gui = GUI()

    # rospy.Subscriber("cam_front", Image, gui.prim_vid.vid_callback())
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
