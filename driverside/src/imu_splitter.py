#!/usr/bin/env python2
import rospy
import yaml
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

# Calls the interpretJoyMsg method of the scheme ControlScheme whenever a joy message is received
# then it sends a twist message and a toggle message for the direction and light respectively


def receive(data):
    trans_x_publisher.publish(data.linear_acceleration.x)
    trans_y_publisher.publish(data.linear_acceleration.y)
    trans_z_publisher.publish(data.linear_acceleration.z)

    rot_x_publisher.publish(data.angular_velocity.x)
    rot_y_publisher.publish(data.angular_velocity.y)
    rot_z_publisher.publish(data.angular_velocity.z)


if __name__ == "__main__":
    trans_x_publisher = rospy.Publisher('/state/trans/x', Float64, queue_size=1)
    trans_y_publisher = rospy.Publisher('/state/trans/y', Float64, queue_size=1)
    trans_z_publisher = rospy.Publisher('/state/trans/z', Float64, queue_size=1)
    rot_x_publisher = rospy.Publisher('/state/rot/x', Float64, queue_size=1)
    rot_y_publisher = rospy.Publisher('/state/rot/y', Float64, queue_size=1)
    rot_z_publisher = rospy.Publisher('/state/rot/z', Float64, queue_size=1)

    try:
        rospy.init_node("ImuSplitter")
        rospy.Subscriber("/imu_data", Imu, receive)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
