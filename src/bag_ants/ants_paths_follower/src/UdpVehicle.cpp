#include <arpa/inet.h>
#include <cstring>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <sys/socket.h>
#include <unistd.h>

#pragma pack(1)
// 传输数据的头部
struct HEADER {
    uint16_t mFrameHeader;//帧头
    int16_t mFrameType;   //帧类型
    uint8_t mFrameLength; //帧长
};
// 自主计算机向底盘发送的操控指令
struct UdpControlCommand {
    HEADER mHeader;
    int16_t vel;      //速度
    int16_t cur;      //曲率
    int16_t pitch;    //俯仰角度
    int16_t frontArm; //前摆臂角度
    int16_t behindArm;//后摆臂角度
    uint8_t flag;     //标志位
    uint8_t checkSum; //校验和
};
// 自主计算机接收的状态
struct UdpRecvCommand {
    HEADER mHeader;
    int16_t vel;          //当前速度
    int16_t cur;          //当前曲率
    int16_t wheelVelLeft; //左轮速度
    int16_t wheelVelRight;//右轮速度
    int16_t frontArm;     //前摆臂角度
    int16_t behindArm;    //后摆臂角度
    uint8_t flag;         //标志位
    int16_t reserve;      //预留
    uint8_t checkSum;     //校验和
};

#pragma pack()
uint8_t sendBuffer[32];//发送缓冲区
uint8_t recvBuffer[32];//接收缓冲区

const int SEND_PORT = 8021;          // 发送数据的UDP端口
const int RECEIVE_PORT = 40021;      // 接收数据的UDP端口
const char *SERVER_IP = "192.2.2.21";// 目标服务器IP地址
const char *CLIENT_IP = "192.2.2.40";// 自身服务器IP地址

int sendSockfd, receiveSockfd;
struct sockaddr_in serverAddr, receiveAddr;


void CmdCallback(const geometry_msgs::TwistStamped::ConstPtr &cmd_vel_) {
    UdpControlCommand dataToSend;
    dataToSend.mHeader.mFrameHeader = 0xffcc;//固定值
    dataToSend.mHeader.mFrameType = 0x11;    //固定值
    dataToSend.mHeader.mFrameLength = 17;    //固定值

    if (cmd_vel_->twist.linear.x == 0.0) {//中心转向，曲率绝对值的大小决定转向的速度
        dataToSend.vel = 0;
        dataToSend.cur = static_cast<int16_t>(cmd_vel_->twist.angular.z * 1000.0);
    } else {//非中心转向，曲率为半径倒数
        dataToSend.vel = static_cast<int16_t>(cmd_vel_->twist.linear.x * 100.0);
        dataToSend.cur = static_cast<int16_t>(cmd_vel_->twist.angular.z / cmd_vel_->twist.linear.x * 1000.0);
    }

    dataToSend.pitch = 0;//不控制俯仰角度
    dataToSend.frontArm = 0;
    dataToSend.behindArm = 0;
    dataToSend.flag = 0;    //自主模式
    dataToSend.checkSum = 0;//赋初值

    uint16_t checkSum = 0;

    memset(&sendBuffer[0], 0, sizeof(sendBuffer));//缓冲区初始化
    memcpy(&sendBuffer[0], &dataToSend, 17);      //将发送的数据存到缓冲区
    // 校验和为前面数据累加和
    for (int i = 0; i < 16; i++) {
        checkSum += sendBuffer[i];
    }

    sendBuffer[16] = checkSum & 0xff;//只保存低八位
    //发送数据
    sendto(sendSockfd, &sendBuffer[0], 17, 0, reinterpret_cast<struct sockaddr *>(&serverAddr), sizeof(serverAddr));
}
//接收数据的函数
void receiveData(int sockfd) {
    while (true) {
        struct sockaddr_in clientAddr;
        socklen_t addr_len = sizeof(clientAddr);

        memset(&recvBuffer[0], 0, sizeof(recvBuffer));//缓冲区初始化
        // 接收数据
        recvfrom(sockfd, &recvBuffer[0], 21, 0, reinterpret_cast<struct sockaddr *>(&clientAddr), &addr_len);

        UdpRecvCommand receivedData;
        memcpy(&receivedData, &recvBuffer[0], 21);//将缓冲区数据取出
        //打印输出
        std::cout << "/* sizeof recvBuffer */" << sizeof(recvBuffer) << std::endl;
        std::cout << "/* message!!! */" << std::endl;
        std::cout << "1:  " << std::hex << static_cast<int>(receivedData.mHeader.mFrameHeader) << std::endl;
        std::cout << "2:  " << static_cast<int>(receivedData.mHeader.mFrameType) << std::endl;
        std::cout << "3:  " << std::dec << static_cast<int>(receivedData.mHeader.mFrameLength) << std::endl;
        std::cout << "4:  " << static_cast<int>(receivedData.vel) << std::endl;
        std::cout << "5:  " << static_cast<int>(receivedData.cur) << std::endl;
        std::cout << "6:  " << static_cast<int>(receivedData.wheelVelLeft) << std::endl;
        std::cout << "7:  " << static_cast<int>(receivedData.wheelVelRight) << std::endl;
        std::cout << "8:  " << static_cast<int>(receivedData.frontArm) << std::endl;
        std::cout << "9:  " << static_cast<int>(receivedData.behindArm) << std::endl;
        std::cout << "10:  " << static_cast<int>(receivedData.flag) << std::endl;
        std::cout << "11:  " << static_cast<int>(receivedData.reserve) << std::endl;
    }
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "udp_send");
    // ros::NodeHandle nh("~");
    ros::NodeHandle nh;                                                 //公共句柄
    ros::Subscriber cmd_sub_ = nh.subscribe("cmd_vel", 5, &CmdCallback);//处理速度消息
    // ros::Publisher feedBack = nh.advertise<geometry_msgs::TwistStamped>("feedback_vel", 5);//将速度发布到cmd_vel中

    // 创建UDP socket用于发送数据
    sendSockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sendSockfd < 0) {
        perror("Error in socket creation for sending");
        exit(1);
    }

    // 创建UDP socket用于接收数据
    receiveSockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (receiveSockfd < 0) {
        perror("Error in socket creation for receiving");
        exit(1);
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SEND_PORT);
    serverAddr.sin_addr.s_addr = inet_addr(SERVER_IP);

    receiveAddr.sin_family = AF_INET;
    receiveAddr.sin_port = htons(RECEIVE_PORT);
    receiveAddr.sin_addr.s_addr = inet_addr(CLIENT_IP);

    //绑定接收端口
    if (bind(sendSockfd, reinterpret_cast<struct sockaddr *>(&receiveAddr), sizeof(receiveAddr)) < 0) {
        perror("Vehicle:Error in binding for sending");
        exit(1);
    }

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        receiveData(sendSockfd);
        loop_rate.sleep();
    }

    return 0;
}
