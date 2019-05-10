#!/usr/bin/env python

import sys
import math
import rospy
from PySide2.QtWidgets import *
from std_msgs.msg import Float32


class Compass(QWidget):
    def __init__(self):
        super(Compass, self).__init__()
        self.pub = rospy.Publisher('compass_offset', Float32, queue_size=25)
        self.init_ui()

    def init_ui(self):
        self.layout = QVBoxLayout()
        self.slider = QSlider()

        self.slider.setMaximum(360)
        self.slider.setMinimum(0)
        self.slider.setTickInterval(1)

        self.layout.addWidget(self.slider)
        self.slider.sliderMoved.connect(self.pub_angle)

        self.setLayout(self.layout)

        self.show()

    def pub_angle(self):
        angle = self.slider.value()
        print angle
        r_angle = (angle * math.pi) / 180
        self.pub.publish(r_angle)


def main():
    app = QApplication([])
    slider_widget = Compass()
    rospy.init_node('CompassPub')

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()