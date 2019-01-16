from ControlScheme import ControlScheme
import rospy
import yaml
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8

# Calls the interpretJoyMsg method of the scheme ControlScheme whenever a joy message is received
# then it sends a twist message and a toggle message for the direction and light respectively
def recieve(data):
	scheme.interpretJoyMsg(data.axes, data.buttons)
	scheme.sendTwistMessage()
	scheme.sendToggleMessage()
	
def recieve2(data):
	print(data)
	print("scheme index : ",scheme.index)
	scheme.setIndex(data.data)
	print(scheme.index)


if __name__ == "__main__":
    scheme = ControlScheme()
    scheme.parseXML()
    try:
        rospy.init_node("ControlHandler")
        rospy.Subscriber("joy", Joy, recieve)
	rospy.Subscriber("gui", Int8, recieve2)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException: pass
