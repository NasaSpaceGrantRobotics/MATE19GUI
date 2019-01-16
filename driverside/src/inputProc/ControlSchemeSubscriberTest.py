import rospy
import yaml
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

"""boolean variable tracking the current value of the light"""
curLight = Bool()
curLight.data = True

def receiveLight(data):
	print(data.data)

def receivePosRot(data):
	print(data)

if __name__ == "__main__":
	try:
		#initialize light node and light subscriber
		rospy.init_node("light_toggle_subscriber")
		rospy.Subscriber("Toggle",Bool,receiveLight)

		rospy.init_node("twist_subscriber")
		rospy.Subscriber("ControlOutput",Twist,receivePosRot)

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			rate.sleep()
	except rospy.ROSInterruptException: pass
