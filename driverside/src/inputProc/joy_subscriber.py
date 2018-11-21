from ControlScheme import ControlScheme
import rospy
import yaml
from sensor_msgs.msg import Joy

# Calls the interpretJoyMsg method of the scheme ControlScheme whenever a joy message is received
# then it sends a twist message and a toggle message for the direction and light respectively
def recieve(data):
    scheme.interpretJoyMsg(data.axes, data.buttons)
    scheme.sendTwistMessage()
    scheme.sendToggleMessage()


if __name__ == "__main__":
    scheme = ControlScheme()
    scheme.parseXML()
    try:
        rospy.init_node("ControlHandler")
        rospy.Subscriber("joy", Joy, recieve)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException: pass