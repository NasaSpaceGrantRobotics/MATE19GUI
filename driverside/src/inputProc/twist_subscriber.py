import rospy
import yaml
from geometry_msgs.msg import Twist

#Calls the interpretJoyMsg method of the test ControlScheme whenever a joy message is received
def recieve (data):
    print(data)

if __name__ == "__main__":
    try:
        rospy.init_node("twist_subscriber")
        rospy.Subscriber("ControlOutput", Twist, recieve)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException: pass