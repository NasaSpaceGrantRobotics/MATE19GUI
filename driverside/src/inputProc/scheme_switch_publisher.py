import rospy
from std_msgs.msg import Int8

if __name__ == "__main__":
	try:
		rospy.init_node("gui")
		publisher = rospy.Publisher("gui",Int8, queue_size=10)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			msg = int(input("scheme num 1 2 :"))
			publisher.publish(msg)
			rate.sleep()
	except rospy.ROSInterruptException: pass
