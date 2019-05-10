#!/usr/bin/env python

import os
import sys
import glob
import rospy
from lxml import etree
from PySide2.QtCore import *
from PySide2.QtWidgets import QApplication, QComboBox, QGridLayout, QHBoxLayout, QLabel, QTabWidget, QVBoxLayout, QWidget
from PySide2.QtGui import QPixmap
from std_msgs.msg import Int8


llist_order = ['LeftTrigger', 'LB', 'Back', 'LeftStickY', 'LeftStickX', 'LeftStick', 'DpadY', 'DpadX']
rlist_order = ['RightTrigger', 'RB', 'Start', 'Y', 'X', 'B', 'A', 'RightStickY', 'RightStickX', 'RightStick']


class ControlPanel(QWidget):
    def __init__(self):
        super(ControlPanel, self).__init__()
        self.icons = []
        self.schemes = []
        self.icons_path = "/home/nasgrds/catkin_ws/src/MATE19/driverside/src/icons/*.png"

        self.load_icons()
        """
        # load scheme xml files into an array
        self.scheme_files = glob.glob('../inputProc/control_schemes/scheme_*.xml')  # path may change depending on location of folder
        
        # extract file name as string from each file path
        for filepath in self.scheme_files:
            head, tail = os.path.split(filepath)    # split file name from file path 
            name = os.path.splitext(tail)[0]    # split tag from file 
            name_and_scheme = (name, filepath)
            self.schemes.append(name_and_scheme)
        """
        # initialize ROS node
        self.pub = rospy.Publisher('control_scheme_index', Int8, queue_size=10)
        rospy.init_node('ControlPanel', anonymous=True)

        self.init_ui()
        self.load_schemes()

    def load_icons(self):
        # load png files into an array
        self.icon_files = glob.glob(self.icons_path)

        # extract file name as string from each file path and create QPixmap from each file, saving them as a tuple
        for filepath in self.icon_files:
            head, tail = os.path.split(filepath)  # split file name from file path
            name = os.path.splitext(tail)[0]  # split tag from file
            pxmp = QPixmap(filepath).scaled(QSize(20, 20), Qt.KeepAspectRatioByExpanding)
            name_and_icon = (name, pxmp)
            self.icons.append(name_and_icon)

    def init_ui(self):
        # create widgets for each view
        self.list_view = ListView(self.icons)
        self.map_view = MapView(self.icons)

        # add each view into its own tab
        self.tab_dlg = QTabWidget()
        self.tab_dlg.addTab(self.list_view, 'List View')
        self.tab_dlg.addTab(self.map_view, 'Map View')

        self.scheme_selector = QComboBox()
        self.scheme_selector.setEditable(False)
        self.scheme_selector.currentIndexChanged.connect(self.change_scheme)

        # set layout for the widget
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.tab_dlg)
        self.layout.addWidget(self.scheme_selector)
        self.setLayout(self.layout)

        self.setWindowTitle('Controls')

    def load_schemes(self):
        # add QComboBox option for each scheme
        for scheme_name in self.schemes:
            self.scheme_selector.addItem(scheme_name[0])
        """
            tree = etree.parse(self.scheme_files[index])
            root = tree.getroot()

            self._scheme_list.append(ButtonTargetDict())

            for axis in root.findall('axis'):
                self.scheme_list[list_idx].button_targets[axis.get('name')] = axis.get('target')

            for button in root.findall('button'):
                self.scheme_list[list_idx].button_targets[button.get('name')] = button.get('target')

            print self._scheme_list[list_idx]  # for debugging
        """
    def change_scheme(self, e):
        # self.list_view.set_scheme(self.schemes[e])
        # self.map_view.set_scheme(self.schemes[e])
        self.pub.publish(e)
        rospy.loginfo("Index published: %d", e)


class ListView(QWidget):
    def __init__(self, icons):
        QWidget.__init__(self)
        self._layout = QHBoxLayout()
        self._llist = QWidget()
        self._rlist = QWidget()
        self._llayout = QVBoxLayout()
        self._rlayout = QVBoxLayout()

        for item in llist_order:
            for pair in icons:
                if pair[0] == item:
                    row = IconAndLabel(pair[1], pair[0], 'L')
                    self._llayout.addWidget(row)
                    break

        for item in rlist_order:
            for pair in icons:
                if pair[0] == item:
                    row = IconAndLabel(pair[1], pair[0], 'L')
                    self._rlayout.addWidget(row)
                    break

        self._llist.setLayout(self._llayout)
        self._rlist.setLayout(self._rlayout)
        self._layout.addWidget(self._llist)
        self._layout.addWidget(self._rlist)
        self.setLayout(self._layout)


class MapView(QWidget):
    def __init__(self, icons):
        QWidget.__init__(self)
        self._diagram = QLabel()
        self._diagram.setPixmap(QPixmap('/home/nasgrds/catkin_ws/src/MATE19/driverside/src/controller_diagram.png').scaled(QSize(372, 250), Qt.KeepAspectRatioByExpanding))
        self._layout = QHBoxLayout()
        self._llist = QWidget()
        self._rlist = QWidget()
        self._llayout = QVBoxLayout()
        self._rlayout = QVBoxLayout()

        for item in llist_order:
            for pair in icons:
                if pair[0] == item:
                    row = IconAndLabel(pair[1], pair[0], 'L')
                    self._llayout.addWidget(row)
                    break

        for item in rlist_order:
            for pair in icons:
                if pair[0] == item:
                    row = IconAndLabel(pair[1], pair[0], 'R')
                    self._rlayout.addWidget(row)
                    break

        self._llist.setLayout(self._llayout)
        self._rlist.setLayout(self._rlayout)
        self._layout.addWidget(self._llist)
        self._layout.addWidget(self._diagram)
        self._layout.addWidget(self._rlist)
        self.setLayout(self._layout)


class IconAndLabel(QWidget):
    def __init__(self, icon, label, orient):
        QWidget.__init__(self)
        self._icon = QLabel()
        self._label = QLabel()
        self._label.setAlignment(Qt.AlignCenter)
        self._icon.setPixmap(icon)
        self._label.setText(label)
        self._layout = QHBoxLayout()
        if orient == 'L':
            self._icon.setAlignment(Qt.AlignRight)
            self._layout.addWidget(self._label)
            self._layout.addWidget(self._icon)
        elif orient == 'R':
            self._icon.setAlignment(Qt.AlignLeft)
            self._layout.addWidget(self._icon)
            self._layout.addWidget(self._label)
        self.setLayout(self._layout)


def main():
    app = QApplication([])

    # create node
    control_panel = ControlPanel()
    control_panel.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
