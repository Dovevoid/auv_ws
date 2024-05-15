#!/usr/bin/env python
import rospy
import time
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode
from std_msgs.msg import Int32
from std_msgs.msg import String


# mavros接口
# 目前测试仅支持0 1 模式
mode_TypeSet = ['Manual','Stabilize','Depth Hold','GUIDED','AUTO']

def mode(mode_type):
    # Set Mode mavros
    print("SET MODE")
    rospy.wait_for_service('/mavros/set_mode')
    try:
        modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        modeResponse = modeService(0, mode_type)
        rospy.loginfo(modeResponse)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def arm():
    print("ARM")
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        armResponse = armService(True)
        rospy.loginfo(armResponse)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def disarm():
    print("DISARM")
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        armService(False)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    rospy.init_node('mavros_set_node', anonymous=True)
    mode(mode_type=mode_TypeSet[0])
    arm()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown() :
        rate.sleep()
    disarm()