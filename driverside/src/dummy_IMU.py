#!/usr/bin/env python2
from ControlScheme import ControlScheme
import rospy
import yaml
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Calls the interpretJoyMsg method of the scheme ControlScheme whenever a joy message is received
# then it sends a twist message and a toggle message for the direction and light respectively


def receive(data):
    transState[0] += data.linear.x * 0.05
    transState[1] += data.linear.y * 0.05
    transState[2] += data.linear.z * 0.05
    rotState[0] += data.angular.x * 0.05
    rotState[1] += data.angular.y * 0.05
    rotState[2] += data.angular.z * 0.05


if __name__ == "__main__":

    transState = [0, 0, 0]
    rotState = [0, 0, 0]

    imuPublisher = rospy.Publisher('/imu_data', Imu, queue_size=10)
    try:
        rospy.init_node("dummy_IMU")
        rospy.Subscriber("/control_effort_final", Twist, receive)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = Imu()
            msg.linear_acceleration.x = transState[0]
            msg.linear_acceleration.y = transState[1]
            msg.linear_acceleration.z = transState[2]
            msg.angular_velocity.x = rotState[0]
            msg.angular_velocity.y = rotState[1]
            msg.angular_velocity.z = rotState[2]
            imuPublisher.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException: pass
