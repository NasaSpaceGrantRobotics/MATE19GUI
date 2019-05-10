import rospy
import yaml
from sensor_msgs.msg import Twist
from std_msgs.msg import Int8


# Calls the interpretJoyMsg method of the scheme ControlScheme whenever a joy message is received
# then it sends a twist message and a toggle message for the direction and light respectively
def recieve(data):
    print("test")

if __name__ == "__main__":
    try:
        rospy.init_node("HL_Planner")
        rospy.Subscriber("joy", Joy, recieve)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
except rospy.ROSInterruptException:
pass
