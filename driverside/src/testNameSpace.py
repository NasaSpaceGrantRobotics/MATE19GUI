#! /usr/bin/env python
import rospy
from std_msgs.msg import Float64

def recieve(data):
        print(data)
	rot_x = data
	print()

if __name__ == "__main__":
        try:
                rospy.init_node("gui")
                rospy.Subscriber("/test/gui1",Float64,recieve)
                rospy.Subscriber("/test /test2/gui2",Float64,recieve)
		rospy.Subscriber("/test /test2/gui3",Float64,recieve)
		publisher = rospy.Publisher("/test/gui1",Float64, queue_size=10)
		publisher = rospy.Publisher("/test /test2/gui2",Float64, queue_size=10)
		publisher = rospy.Publisher("/test /test2/gui3",Float64, queue_size=10)

		rate = rospy.Rate(10)
                while not rospy.is_shutdown():
                        msg = int(input("scheme num 1 2 :"))
                        publisher.publish(msg)
                        rate.sleep()
        except rospy.ROSInterruptException: pass

