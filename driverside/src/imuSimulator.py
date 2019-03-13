
import rospy
from sensor_msgs.msg import Imu

def recieve(data):
        print(data.linear_acceleration.x)

if __name__=="__main__":
        try:
		rospy.init_node("imu_data")
		rospy.Subscriber("imuSplitter",Imu,recieve)
		publisher = rospy.Publisher("imuSplitter",Imu, queue_size=10)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
                        msg = Imu()
			publisher.publish(msg)
                        rate.sleep()
        except rospy.ROSInterruptException: pass

