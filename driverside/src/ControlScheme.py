#!/usr/bin/env python2
# the indices of values from joy message:
# index number of / joy.buttons:
# 0 - A
# 1 - B
# 2 - X
# 3 - Y
# 4 - LB
# 5 - RB
# 6 - back
# 7 - start
# 8 - power
# 9 - Button stick left
# 10 - Button stick right

# index number of / joy.axis:
# 0 - Left / Right Axis stick left
# 1 - Up / Down Axis stick left
# 2 - LT
# 3 - Left / Right Axis stick right
# 4 - Up / Down Axis stick right
# 5 - RT
# 6 - cross key left / right
# 7 - cross key up / down
from lxml import etree
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import glob


class ControlScheme:
    def __init__(self, directory):
        """
            initializes an instance of a control scheme
        """
        # this will contain an 2D array of different control schemes where the nth control scheme will be designated
        # from axesTarget[n] and the target for the mth index axis will be axesTarget[n][m]
        self.axes_target = []
        self.buttons_target = []

        self.current_light = Bool()
        self.current_light.data = False
        self.previous_light = 0

        self.button_names = {
            "A": 0,
            "B": 1,
            "X": 2,
            "Y": 3,
            "LB": 4,
            "RB": 5,
            "Back": 6,
            "Start": 7,
            "LeftStick": 8,
            "RightStick": 9
        }
        self.axes_names = {
            "LeftStickX": 0,
            "LeftStickY": 1,
            "LeftTrigger": 2,
            "RightStickX": 3,
            "RightStickY": 4,
            "RightTrigger": 5,
            "DpadX": 6,
            "DpadY": 7
        }

        # Dictionary created whenever a joy message is received that matches a target control
        # designated by the ControlScheme with a value from the joy message
        self.target_controls = {}

        # Array of all of the different ControlScheme files to be parsed
        self.XML_file_names = glob.glob(directory + "/scheme_*.xml")
        self.index = 0

        self.trans_X_publisher = rospy.Publisher('setpoint/trans/x', Float64, queue_size=1)
        self.trans_Y_publisher = rospy.Publisher('setpoint/trans/y', Float64, queue_size=1)
        self.trans_Z_publisher = rospy.Publisher('setpoint/trans/z', Float64, queue_size=1)
        self.rot_X_publisher = rospy.Publisher('setpoint/rot/x', Float64, queue_size=1)
        self.rot_y_publisher = rospy.Publisher('setpoint/rot/y', Float64, queue_size=1)
        self.rot_Z_publisher = rospy.Publisher('setpoint/rot/z', Float64, queue_size=1)
        self.toggle_publisher = rospy.Publisher('toggle', Bool, queue_size=1)

    # Parses all of the xml files with names in the XMLfileNames array and creates an array
    # of axes and buttons to append to the axesTarget and buttonsTarget arrays respectively
    # This is done so that all of the xml files can be read in at the same time and
    # switching between them can be done by switching the index
    def parse_xml(self):
        """
        Parse through all xml files in XMLfileNames to create control scheme
        """
        for file_name in self.XML_file_names:
            tree = etree.parse(file_name)
            root = tree.getroot()

            axes = [None]*8
            buttons = [None]*11

            for axis in root.findall("axis"):
                axes[self.axes_names[axis.get("name")]] = axis.get("target")

            for button in root.findall("button"):
                buttons[self.button_names[button.get("name")]] = button.get("target")

            self.axes_target.append(axes)
            self.buttons_target.append(buttons)

    # Populates the dictionary of targetControls by matching the incoming values with the designated targets
    def interpret_joy_msg(self, axes_values, buttons_values):
        """
        Populates the dictionary of targetControls by matching the incoming
        values with the designated targets

        Keyword arguments:

        axes_values -- Incoming axes values coming from 360 controller
        buttons_values -- Incoming buttons values from 360 controller
        """
        for i in range(len(axes_values)):
            if self.axes_target[self.index][i] is not None:
                self.target_controls[self.axes_target[self.index][i]] = axes_values[i]

        for i in range(len(buttons_values)):
            if self.buttons_target[self.index][i] is not None:
                self.target_controls[self.buttons_target[self.index][i]] = buttons_values[i]

    # changes the index of control schemes
    def set_index(self, n):
        """
        changes the index of control schemes
        Keyword arguments:

        n -- the index value of control scheme to be changed to
        """
        if 0 <= n < len(self.axes_target) and n < len(self.buttons_target):
            self.index = n

    def send_target_message(self):
        """
        publish twist message with linear x,y,z and angular x,y,z
        """
        self.trans_X_publisher.publish(self.target_controls["trans_x"])
        self.trans_Y_publisher.publish(self.target_controls["trans_y"])
        self.trans_Z_publisher.publish(self.target_controls["trans_z"])
        self.rot_X_publisher.publish(self.target_controls["rot_x"])
        self.rot_y_publisher.publish(self.target_controls["rot_y"])
        self.rot_Z_publisher.publish(self.target_controls["rot_z"])

    def send_toggle_message(self):
        """
        Publish boolean message indicating the current state of light
        """
        if not (self.target_controls["light"] == self.previous_light):

            if self.target_controls["light"] == 1:
                self.current_light.data = not self.current_light.data
                self.toggle_publisher.publish(self.current_light)
        self.previous_light = self.target_controls["light"]