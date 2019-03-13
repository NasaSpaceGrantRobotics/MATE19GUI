#! /usr/bin/env python
import rospy
from std_msgs.msg import Float64

def recieve(data):
	print(data)

if __name__ == "__main__":
	try:
		rospy.init_node("gui")
		rospy.Subscriber("/test/GuiS",Float64,recieve)
		publisher = rospy.Publisher("gui",Float64, queue_size=10)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			msg = int(input("scheme num 1 2 :"))
			publisher.publish(msg)
			rate.sleep()
	except rospy.ROSInterruptException: pass
