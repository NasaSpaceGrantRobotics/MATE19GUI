#!/usr/bin/env python2
import rospy
import yaml
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

# Calls the interpretJoyMsg method of the scheme ControlScheme whenever a joy message is received
# then it sends a twist message and a toggle message for the direction and light respectively


def receive(data):
    transXPublisher.publish(data.linear_acceleration.x)
    transYPublisher.publish(data.linear_acceleration.y)
    transZPublisher.publish(data.linear_acceleration.z)

    rotXPublisher.publish(data.angular_velocity.x)
    rotYPublisher.publish(data.angular_velocity.y)
    rotZPublisher.publish(data.angular_velocity.z)


if __name__ == "__main__":
    transXPublisher = rospy.Publisher('/state/trans/x', Float64, queue_size=10)
    transYPublisher = rospy.Publisher('/state/trans/y', Float64, queue_size=10)
    transZPublisher = rospy.Publisher('/state/trans/z', Float64, queue_size=10)
    rotXPublisher = rospy.Publisher('/state/rot/x', Float64, queue_size=10)
    rotYPublisher = rospy.Publisher('/state/rot/y', Float64, queue_size=10)
    rotZPublisher = rospy.Publisher('/state/rot/z', Float64, queue_size=10)

    try:
        rospy.init_node("IMUsplitter")
        rospy.Subscriber("/imu_data", Imu, receive)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
