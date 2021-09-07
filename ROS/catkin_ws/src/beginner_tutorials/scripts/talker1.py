import rospy
from rospy.topics import Publisher
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import LaserScan
import time
import math
import numpy as np

arry = []
ArcadeDriveArry = Int32MultiArray(data=arry)
pub = rospy.Publisher('ArcadeDrive', Int32MultiArray, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(100)
addvalue = 5


def ArcadeDrive(y, x, z):
    ArcadeDriveArry.data = [
        y + x - z,
        y - x + z,
        y + x - z,
        y - x + z]
    pub.publish(ArcadeDriveArry)


def callback3(data):
    maxvalue = max(data.ranges)
    angle = data.ranges.index(maxvalue)
    rospy.loginfo(angle)
    if 0 < angle <= 180:
        ArcadeDrive(20, -int(maxvalue)*addvalue, 0)
    elif 180 < angle <= 360:
        ArcadeDrive(20, int(maxvalue)*addvalue, 0)
    else:
        ArcadeDrive(0, 0, 0)

def talker():

    while not rospy.is_shutdown():
        rospy.Subscriber("scan", LaserScan, callback3)
        rate.sleep()
        time.sleep(10)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        ArcadeDrive(0, 0, 0)