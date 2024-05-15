/* function: create a tcp server to listen the info from the client */


#include <ros/ros.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <std_msgs/String.h>


#define MYPORT 1140   //port 
#define BUF_SIZE 1024 //the buffer size
#define LOG  1


int main (int argc, char** argv)
{
	ros::init(argc, argv, "nuc_tcpserver_node");
	ros::NodeHandle nh;

    ros::Publisher nuc_tcpsever_pub = nh.advertise<std_msgs::String>("servo_controlsignal", 10);
    

	/*
	 *@fuc: 监听套节字描述符
	 *@fuc; 服务器端IP4地址信息,struct关键字可不要
	 */
	int server_fd;
	struct sockaddr_in server_address;
	
	/*
	 *@fuc: 使用socket()函数产生套节字描述符
	 */
	server_fd = socket(AF_INET, SOCK_STREAM, 0);
	if(server_fd == -1)
	{
		ROS_ERROR_STREAM("creat server_fd error");
		return -1;
	}
	
	/*
	 *@fuc: 初始化server套节字地址信息 
	 */
	memset((void *)&server_address,0,sizeof(server_address));
	server_address.sin_family = AF_INET;
	server_address.sin_addr.s_addr = htonl(INADDR_ANY);
	server_address.sin_port = htons(MYPORT);
 
 	/*
	 *@fuc: 用bind()函数，将套接字与指定的协议地址绑定 
	 */
	if(bind(server_fd,(struct sockaddr *)&server_address,sizeof(server_address)) < 0)
	{
		ROS_ERROR_STREAM("bind server_fd error");
		return -1;
	}
	
 	/*
	 *@fuc: 使用listen()函数，等待客户端的连接 
	 */
    if(listen(server_fd, LOG) < 0)
    {
        ROS_ERROR_STREAM("listen the client error");
        return -1;
    }
    
    ROS_INFO_STREAM("waiting for the client connected");

	/*
	 *@fuc: client socket
	 *@fuc; client端IP4地址信息,struct关键字可不要
	 */
    int client_fd;
    struct sockaddr_in client_address;
    socklen_t addrlen = sizeof(client_address);

    client_fd = accept(server_fd, (struct sockaddr *)&client_address, &addrlen);
    //默认堵塞直到一个tcp客户端连接到服务器
    if (client_fd < 0)
    {
        ROS_ERROR_STREAM("connect error ");
        return -1;
    }
    printf("You got a connection from client's IP is %s, port is %d\n",
           inet_ntoa(client_address.sin_addr), ntohs(client_address.sin_port));

    
    //不断监听客户端请求
    int recv_size;
    char buffer[BUF_SIZE];
	//ros::Rate rate = 10;
	while(ros::ok)
	{
        memset(buffer, 0, BUF_SIZE);
        // 接收客户端发送的消息
        recv_size = recv(client_fd, buffer,BUF_SIZE, 0);
        if (recv_size > 0)
        {
            // 打印接收到的消息
            std_msgs::String msg;
            msg.data = buffer;
            ROS_INFO("Received message from client: %s", msg.data.c_str());
            nuc_tcpsever_pub.publish(msg);
        }
		//ros::spinOnce();
        //rate.sleep();  // 控制循环频率
	}

    // 关闭客户端socket
    close(client_fd);
    close(server_fd);
	return 0;
}