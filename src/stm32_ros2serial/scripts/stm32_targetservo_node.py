#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String

serial_port = "/dev/ttyUSB0"
serial_baudrate = 115200

# 串口变量
ser = serial.Serial()

#模式设置 0 是自主 1 是遥控
MODE = 0

# 打开串口
def openserial():
    try:
        ser.port = serial_port  # 设备端口号
        ser.baudrate = serial_baudrate  # 波特率
        ser.timeout = 1  # 超时时间为1秒
        ser.open()  # 打开串口
    except serial.SerialException:
        rospy.logerr("Unable to open port")
        return

    if ser.is_open:
        rospy.loginfo("OPEN")


# callback: send the command to STM32
def sendToSTM32(msg):
    rospy.loginfo("Info is sending: %s", msg.data)
    # 发送数据
    stm32_command = msg.data
    data = [ord(c) for c in stm32_command]
    ser.write(data)
    # stm32需要 \r\n
    data_n = b"\r\n"
    ser.write(data_n)


def main():
    rospy.init_node("stm32_serialsend_node")
    m = rospy.NodeHandle()

    # 打开串口
    openserial()

    global MODE
    if MODE == 0:
        #自主机械臂控制
        stm32_autosub = rospy.Subscriber("targetAngle_topic", String, sendToSTM32, queue_size=10)
    else:
        stm32_sub = rospy.Subscriber("servo_controlsignal", String, sendToSTM32, queue_size=10)

    rate = rospy.Rate(10)  # 设置循环频率为10Hz
    while not rospy.is_shutdown():
        # 以下代码作测试用，显示单片机传输来的信息
        w = ser.in_waiting  # ser.read缓冲区中的字节数
        received_data = []

        if w != 0:
            received_data = ser.read(w)
            # 将接收到的数据转换为字符串并打印
            received_message = received_data.decode("utf-8")
            rospy.loginfo("the stm32_back is %s", received_message)
            w = 0

        rospy.spinOnce()
        rate.sleep()  # 控制循环频率


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass