import socket
import serial
import time

# nuc的IP+端口
nuc_ip = "192.168.1.3"
nuc_com = 1140

# 串口
# 使用F407主板  /dev/ttyUSB0  ，使用自制主板  /dev/ttyACM0
serial_port = '/dev/ttyUSB0'
serial_baudrate = 115200


def send_data(client_socket, data):
    try:
        client_socket.send(data)
        print("Data sent successfully")
    except Exception as e:
        print(f"Failed to send data: {e}")


# 初始化客户端和连接网络
client_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_add = (nuc_ip, nuc_com)
client_s.connect(server_add)

# 初始化串口
ser = serial.Serial(serial_port, serial_baudrate, timeout=0.1)  # com口 波特率 延时
time.sleep(0.5)

# 执行程序
try:
    while True:
        Read = ser.readline()  # 串口读到的数据放到Read中
        if Read:
            send_data(client_s, Read)  # 使用回调函数发送数据
        else:
            print("No data read from serial port")
except KeyboardInterrupt:
    print("Program terminated by user")
finally:
    client_s.close()
    ser.close()
