#!/usr/bin/env python

import sys
import rospy
import cv2
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow, QAction, qApp
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class GUI(QMainWindow):
    def __init__(self):
        super(GUI, self).__init__()

        self.initUI()

    def initUI(self):
        # setup menubar
        menubar = self.menuBar()

        # setup file menu and all its items
        file_menu = menubar.addMenu('&File')
        quit_item = QAction('&Quit', self)
        quit_item.setStatusTip('Quit Application')
        quit_item.triggered.connect(qApp.quit)
        file_menu.addAction(quit_item)

        # setup debug menu and all its items
        debug_menu = menubar.addMenu('&Debug')

        # setup controls menu and all its items
        controls_menu = menubar.addMenu('&Controls')

        # setup status bar
        self.statusBar()

        self.setWindowTitle('ASU MATE 2019')
        self.showMaximized()


def main():
    app = QApplication([])
    gui = GUI()
    #rospy.init_node('gui')
    #rospy.spin()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()