#!/usr/bin/python
from os import WIFCONTINUED
import rospy
from rospy.topics import Publisher
from std_msgs.msg import String, Int32MultiArray
import time

arry = []
ArcadeDriveArry = Int32MultiArray(data=arry)
pub = rospy.Publisher('ArcadeDrive', Int32MultiArray, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(100)


def up():
    ArcadeDriveArry.data = [30,0,0]
    pub.publish(ArcadeDriveArry)
    print("up")
def down():
    ArcadeDriveArry.data = [-30,0,0]
    pub.publish(ArcadeDriveArry)
    print("down")
def stop():
    ArcadeDriveArry.data = [0,0,0]
    pub.publish(ArcadeDriveArry)
    print("stop")

def talker():
    ArcadeDriveArry.data = [0,0,0]
    while not rospy.is_shutdown():
        up()
        time.sleep(1)
        down()
        time.sleep(1)
        stop()
        time.sleep(1)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
