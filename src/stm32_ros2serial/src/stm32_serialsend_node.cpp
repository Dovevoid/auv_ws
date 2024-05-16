/* function: ros chat with stm32 through USB */
/* reference: csdn blog: https://blog.csdn.net/qq_19324147/article/details/105669169 */


#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

#define serial_port "/dev/ttyUSB0"
#define serial_baudrate 115200

/* 串口变量 */ 
serial::Serial ser;


/* 打开串口 */
void openserial()
{
    try
    {
        ser.setPort(serial_port);                           // 设备端口号
        ser.setBaudrate(serial_baudrate);                      // 波特率
        serial::Timeout t = serial::Timeout::simpleTimeout(1000); // overtime
        ser.setTimeout(t);
        ser.open();                                               // 打开串口
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
    }
    
    if (ser.isOpen())
    {
        ROS_INFO("OPEN");
    }
}


/* callback: send the command to STM32 */
void sendToSTM32(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Info is sending, %s",msg->data.c_str());
    // 发送数据
    std::string stm32_command = msg->data;
    std::vector<uint8_t> data(stm32_command.begin(), stm32_command.end());
    ser.write(data);
    //stm32 need \r\n
    std::string data_n = "\r\n";
    ser.write(data_n);
  
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "stm32_serialsend_node");
    ros::NodeHandle n_area("~");
    ros::NodeHandle m;
    //注意使用判断必须使用全局sub
    ros::Subscriber stm32_sub ;
    // 打开串口
    openserial(); 

    //舵机模式控制 0是自主 1是遥控 默认为1
    int mode_flag;
    //注意要使用局部句柄才能传参
    n_area.param("mode_flag", mode_flag,1); // 读取 mode_flag 参数,如果未设置则默认为 1

    if (mode_flag == 0)
    {
        // 自主抓取
        stm32_sub = m.subscribe<std_msgs::String>("targetAngle_topic", 10, sendToSTM32);
        ROS_INFO("auto servo_control");
    }
    else
    {
        // 遥控抓取
        stm32_sub = m.subscribe<std_msgs::String>("servo_controlsignal", 10, sendToSTM32);
        ROS_INFO("human servo_control");
    }
    
    // 自主抓取
    // ros::Subscriber stm32_sub = m.subscribe<std_msgs::String>("targetAngle_topic", 10, sendToSTM32);

    ros::Rate rate(10);  // 设置循环频率为10Hz
    while (ros::ok())
    {
        //以下代码作测试用,显示单片机传输来的信息
        size_t w = ser.available();//the count of ser.read buffer
        std::vector<uint8_t> received_data;

        if (w!=0)
        {
            ser.read(received_data,w);
            // 将接收到的数据转换为字符串并打印
            std::string received_message(received_data.begin(), received_data.end());
            ROS_INFO("the stm32_back is %s",received_message.c_str());
            w=0;
        }        
        
        ros::spinOnce();
        rate.sleep();  // 控制循环频率
    }
    
    return 0;
}
