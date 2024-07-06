#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "ros/ros.h"
// #include "tank_sdk/TankFeedback.h"
// #include "tank_sdk/TankCMD.h"

#include <geometry_msgs/TwistStamped.h>

#pragma pack(1)

struct HEADER {
    uint16_t mFrameHeader;
    int16_t mFrameType;
    uint8_t mFrameLength;
};

struct UdpControlCommand {
    HEADER mHeader;
    int16_t vel;
    int16_t cur;
    int16_t pitch;
    int16_t frontArm;
    int16_t behindArm;
    uint8_t flag;
    uint8_t checkSum;
};

struct UdpRecvCommand {
    HEADER mHeader;
    int16_t vel;
    int16_t cur;
    int16_t wheelVelLeft;
    int16_t wheelVelRight;
    int16_t frontArm;
    int16_t behindArm;
    uint8_t flag;
    int16_t reserve;
    uint8_t checkSum;
};

#pragma pack()
uint8_t sendBuffer[32];
uint8_t recvBuffer[32];

const int SEND_PORT = 8021;          // 发送数据的UDP端口
const int RECEIVE_PORT = 40021;      // 接收数据的UDP端口
const char *SERVER_IP = "192.2.2.21";// 目标服务器IP地址

int sendSockfd, receiveSockfd;
struct sockaddr_in serverAddr, receiveAddr;

ros::Subscriber cmd_sub_;
ros::Publisher feedback_pub_;
ros::Timer feedback_timer_;

// void TankCmdCallback(const tank_sdk::TankCMD::ConstPtr &msg)
// {

//     UdpControlCommand dataToSend;
//     dataToSend.mHeader.mFrameHeader = 0xffcc;
//     dataToSend.mHeader.mFrameType = 0x11;
//     dataToSend.mHeader.mFrameLength = 12;
//     // old. work
//     // dataToSend.mHeader.mFrameLength = 16;

//     if (fabs(msg->velocity) < 0.05)
//     {
//         // ???? 1000.0 ????
//         dataToSend.vel = 0;
//         dataToSend.cur = (int16_t) (msg->angle_velocity * 1000.0);
//     }
//     else
//     {
//         dataToSend.vel = (int16_t) (msg->velocity * 100.0);
//         dataToSend.cur = (int16_t) (msg->angle_velocity * 1.8 / msg->velocity * 1000.0);
//     }

//     dataToSend.pitch = 0;

//     uint16_t checkSum = 0;

//     memset(&sendBuffer[0], 0, sizeof(sendBuffer));
//     memcpy(&sendBuffer[0], &dataToSend, 11);

//     for (int i = 0; i < 11; i++)
//     {
//         checkSum += sendBuffer[i];
//     }

//     sendBuffer[11] = checkSum & 0xff;

//     sendto(sendSockfd, &sendBuffer[0], 12, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
// }

void TankCmdCallback(const geometry_msgs::TwistStamped::ConstPtr &cmd_vel_) {
    std::cout << "TankCmdCallback" << std::endl;
    UdpControlCommand dataToSend;
    dataToSend.mHeader.mFrameHeader = 0xffcc;
    dataToSend.mHeader.mFrameType = 0x11;
    dataToSend.mHeader.mFrameLength = 17;
    // old. work
    // dataToSend.mHeader.mFrameLength = 16;

    if (fabs(cmd_vel_->twist.linear.x) < 0.05) {
        // ???? 1000.0 ????
        dataToSend.vel = 0;
        dataToSend.cur = (int16_t) (cmd_vel_->twist.angular.z * 1000.0);
    } else {
        dataToSend.vel = (int16_t) (cmd_vel_->twist.linear.x * 100.0);
        dataToSend.cur = (int16_t) (cmd_vel_->twist.angular.z * 1.8 / cmd_vel_->twist.linear.x * 1000.0);
    }

    dataToSend.pitch = 0;
    dataToSend.frontArm = 0;
    dataToSend.behindArm = 0;
    dataToSend.flag = 0;

    uint16_t checkSum = 0;

    memset(&sendBuffer[0], 0, sizeof(sendBuffer));
    memcpy(&sendBuffer[0], &dataToSend, 16);

    for (int i = 0; i < 16; i++) {
        checkSum += sendBuffer[i];
    }

    sendBuffer[16] = checkSum & 0xff;

    sendto(sendSockfd, &sendBuffer[0], 17, 0, (struct sockaddr *) &serverAddr, sizeof(serverAddr));
}

void feedbackCallback(const ros::TimerEvent &e) {
    struct sockaddr_in clientAddr;
    socklen_t addr_len = sizeof(clientAddr);

    memset(&recvBuffer[0], 0, sizeof(recvBuffer));
    recvfrom(sendSockfd, &recvBuffer[0], 16, 0, (struct sockaddr *) &clientAddr, &addr_len);

    uint16_t checkSum = 0;
    for (int i = 0; i < 15; i++) {
        checkSum += recvBuffer[i];
    }
    checkSum = checkSum & 0xff;

    // std::cout << "/* message */" <<checkSum<< std::endl;
    // if ((int)recvBuffer[15] == (int)(checkSum ))
    if (true) {

        UdpRecvCommand receivedData;
        memcpy(&receivedData, &recvBuffer[0], 16);
        std::cout << "1:  " << std::hex << static_cast<int>(receivedData.mHeader.mFrameHeader) << std::endl;
        std::cout << (int) (receivedData.frontArm) << std::endl;
        std::cout << (int) (receivedData.behindArm) << std::endl;
        // std::cout << "Received Data->vel: " << receivedData.vel << " cur: " << receivedData.wheelVelLeft << std::endl;
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "udp_send");
    // ros::NodeHandle nh("~");
    ros::NodeHandle nh;

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
    receiveAddr.sin_addr.s_addr = INADDR_ANY;

    //绑定接收端口
    if (bind(sendSockfd, (struct sockaddr *) &receiveAddr, sizeof(receiveAddr)) < 0) {
        perror("Vehicle:Error in binding for sending");
        exit(1);
    }

    // ROS
    cmd_sub_ = nh.subscribe("cmd_vel", 5, &TankCmdCallback);
    // feedback_pub_ = nh.advertise<tank_sdk::TankFeedback>("feed_back", 5);
    feedback_timer_ = nh.createTimer(ros::Duration(0.01), feedbackCallback);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
