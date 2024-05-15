#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import math
import numpy as np

####################################关节角度解算##########################################
# 定义符号常量
PI = math.pi
#定义关节参数，单位cm
l1, l2, l3 = 16, 31, 20

# 将角度转换为弧度
def deg_to_rad(deg):
    return deg * PI / 180

# 将弧度转换为角度
def rad_to_deg(rad):
    return rad * 180 / PI

# DH参数的摄像头坐标系向基坐标转换
def dh_transform(a, alpha, d, theta):
    """
    Denavit-Hartenberg变换矩阵
    :param a: 连杆长度
    :param alpha: 连杆扭转角
    :param d: 连杆偏移
    :param theta: 关节角度
    :return: 4x4 变换矩阵
    """
    return np.array([
        [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
        [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
        [0, math.sin(alpha), math.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# 3R机械臂逆运动学计算
def inverse_kinematics_3R(xt, yt, zt, l1, l2, l3):
     # 判断可达工作空间
    zt = zt + l1
    d = math.sqrt(xt**2 + yt**2+(zt-l1)**2) 
    if d > l2 + l3 or d < abs(l2 - l3):
        raise ValueError("目标点在机械臂的可达范围之外。")
    
    # 计算theta1
    theta1 = math.atan2(yt, xt)
    # 计算theta3
    theta3 =math.acos((xt**2+yt**2+(zt-l1)**2-l2**2-l3**2)/(2*l2*l3))  
    # 计算theta2
    theta2 = math.atan2((zt-l1)*(l2+l3*math.cos(theta3))*np.cos(theta1) -xt*l3*np.sin(theta3),xt*(l2+l3*np.cos(theta3))+(zt-l1)*l3*np.sin(theta3)*np.cos(theta1))
    
    # 转换为角度
    theta1_deg = rad_to_deg(theta1)
    theta2_deg = rad_to_deg(theta2)
    theta3_deg = rad_to_deg(theta3)

    return theta1_deg, theta2_deg, theta3_deg

########################### 进行逆结算和格式转换 ################################

def point2command(msg, pub):
    global l1,l2,l3
    data = msg.data.split(',')
    angle = inverse_kinematics_3R(eval(data[0]),eval(data[1]),eval(data[2]),l1,l2,l3)
    #存储数据数组
    splitted_data = []
    for i in range(len(angle)):
        int_angle = int(angle[i]) 
        #确定符号位
        if int_angle >0:
            sign_s = '0'
        else:
            sign_s = '1'
        #补零
        str_len =len(str(abs(int_angle)))
        zero_str = (3-str_len)*'0'
        #数据格式
        splitted_data.append(sign_s + zero_str + str(abs(int_angle))) 

    #打包数据帧格式 0101 角度1 0101 角度2 0101 角度三 0101*4
    const_stringzym = '0101'
    command = ''
    command = const_stringzym + splitted_data[0] + const_stringzym + splitted_data[1] + \
              const_stringzym + splitted_data[2] + 4*const_stringzym

    # 发布转换后的消息到 "targetAngle_topic" 话题
    msg = String()  #创建 msg 对象
    msg.data = command 
    rospy.loginfo("写出的数据:%s",msg.data)
    pub.publish(msg)

def publisher():
    # 初始化ROS节点
    rospy.init_node('servo_targetpoint_pub_node', anonymous=True)
    servo_targetpoint_pub = rospy.Publisher('targetpoint_topic', String, queue_size=10)
    # 创建一个发布者，发布类型为String的消息到名为 "targetAngle_topic" 的话题
    pub = rospy.Publisher('targetAngle_topic', String, queue_size=10)
    servo_targetpoint_sub = rospy.Subscriber("targetpoint_topic", String, point2command, callback_args=pub, queue_size=10)

    # 设置发布频率为10Hz
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 按照设定的频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

