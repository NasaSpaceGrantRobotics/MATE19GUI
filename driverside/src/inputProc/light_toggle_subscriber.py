from ControlScheme import ControlScheme
import rospy
import yaml
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

curLight = Bool()
curLight.data = True
"""
*Subscribe from joystick topic
"""


def receive(data):
    print(data.data)


if __name__=="__main__":
   try:
		rospy.init_node("light_toggle_subscriber")
		rospy.Subscriber("Toggle",Bool,receive)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
		    rate.sleep()
	except rospy.ROSInterruptException: pass
