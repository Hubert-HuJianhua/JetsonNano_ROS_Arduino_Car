#!/usr/bin/python
from os import WIFCONTINUED
import rospy
from rospy.topics import Publisher
from std_msgs.msg import String, Int32MultiArray


def talker():
    pub = rospy.Publisher('chatter', Int32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        a = [20,50,60]
        pub.publish(a)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
