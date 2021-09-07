#!/usr/bin/python
from rosgraph.names import anonymous_name
import rospy
from std_msgs.msg import String, Int32MultiArray

def callback1(data):
    rospy.loginfo(data.data)

def callback2(data):
    rospy.loginfo(data.data[0])
    rospy.loginfo(data.data[1])
    rospy.loginfo(data.data[2])

def listener():
    rospy.init_node('liserner', anonymous=True)
    rospy.Subscriber("button_press", String, callback1)
    rospy.Subscriber("ArcadeDrive", Int32MultiArray, callback2)
    rospy.spin()


if __name__ == "__main__":
    listener()