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
	if(curLight.data != data.data):
		curLight.data = data.data
		print(curLight)
	else:
		print("already at that value")
#	print("X:",str(data.linear.x),"Y:",str(data.linear.y),"Z:",str(data.linear.z)) 

if __name__=="__main__":
	try:
		rospy.init_node("InputProc")
		rospy.Subscriber("joy",Bool,receive)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			rate.sleep()
	except rospy.ROSInterruptException: pass
