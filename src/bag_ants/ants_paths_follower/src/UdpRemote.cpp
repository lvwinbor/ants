#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#pragma pack(1)
// 传输数据的头部
struct HEADER {
    uint16_t mFrameHeader;//帧头
    uint8_t mFrameType;   //帧类型
    uint8_t mFrameLength; //帧长
};

// 操控端向底盘发送的操控指令
struct UdpControlCommand {
    HEADER mHeader;
    u_char driveGear;                //驱动档位、摆臂档位及摆臂控制
    u_char driveMode;                //操控模式切换
    u_char carMaxSpeed;              //底盘限速
    u_char armMaxSpeed;              //摆臂限速设定
    char chassisControlArmHorizontal;//底盘操纵摇杆水平方向值（右摇杆）
    char chassisControlArmVertical;  //底盘操纵摇杆垂直方向值（右摇杆）
    u_char chassisPower;             //底盘配电
    int16_t frontArmAngle;           //前摆臂角度
    int16_t behindArmAngle;          //后摆臂角度
    u_char dataDestruction;          //数据销毁
    uint16_t reserved;               //预留
    u_char checkSum;                 //校验和
};
// 操控端接收的状态
struct UdpRecvCommand {
    HEADER mHeader;
    uint8_t carDriveState; //平台机动状态
    int8_t carSpeed;       //平台速度
    uint8_t batteryVoltage;//电池电压（V）
    uint8_t batterySoc;    //电池Soc
    uint8_t carDriveMode;  //底盘驾驶模式
    int16_t batteryCircuit;//电池电流
};

#pragma pack()
uint8_t sendBuffer[32];//发送缓冲区
uint8_t recvBuffer[32];//接收缓冲区

const int SEND_PORT = 8021;          // 发送数据的目标UDP端口
const int RECEIVE_PORT = 20033;      // 接收数据的自身UDP端口
const char *SERVER_IP = "192.2.2.21";// 目标服务器IP地址
const char *CLIENT_IP = "192.2.2.40";// 自身服务器IP地址

// 发送数据的线程函数
void sendThread(int sockfd, const sockaddr_in &serverAddr) {

    while (true) {

        UdpControlCommand dataToSend;
        dataToSend.mHeader.mFrameHeader = 0xffaa;//固定值
        dataToSend.mHeader.mFrameType = 0x0a;    //固定值
        dataToSend.mHeader.mFrameLength = 19;    //固定值

        // dataToSend.driveGear = 0b10100111;
        dataToSend.driveGear = 0b0101;             //驱动抵挡、摆臂低档
        dataToSend.driveMode = 0b01;               //自主模式
        dataToSend.carMaxSpeed = 20;               //固定值
        dataToSend.armMaxSpeed = 10;               //固定值
        dataToSend.chassisControlArmHorizontal = 0;//摇杆归零
        dataToSend.chassisControlArmVertical = 0;  //摇杆归零
        dataToSend.chassisPower = 0b1010001;       //强电上电、底盘使能、自主系统上电
        dataToSend.frontArmAngle = 0;              //前摆臂零度
        dataToSend.behindArmAngle = 0;             //后摆逼零度
        dataToSend.dataDestruction = 0;            //固定值
        dataToSend.reserved = 0;                   //固定值
        dataToSend.checkSum = 0;                   //赋初值

        uint16_t checkSum = 0;

        memset(&sendBuffer[0], 0, sizeof(sendBuffer));//缓冲区初始化
        memcpy(&sendBuffer[0], &dataToSend, 19);      //将发送的数据存到缓冲区
        // 校验和为前面数据累加和
        for (int i = 0; i < 18; i++) {
            checkSum += sendBuffer[i];
        }

        sendBuffer[18] = checkSum & 0xff;//只保存低八位
        //发送数据
        sendto(sockfd, &sendBuffer[0], 19, 0, reinterpret_cast<const struct sockaddr *>(&serverAddr), sizeof(serverAddr));

        // 延迟50毫秒
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// 接收数据的线程函数
void receiveThread(int sockfd) {
    while (true) {
        struct sockaddr_in clientAddr;
        socklen_t addr_len = sizeof(clientAddr);

        memset(&recvBuffer[0], 0, sizeof(recvBuffer));//缓冲区初始化
        // 接收数据
        recvfrom(sockfd, &recvBuffer[0], 11, 0, reinterpret_cast<struct sockaddr *>(&clientAddr), &addr_len);

        UdpRecvCommand receivedData;
        memcpy(&receivedData, &recvBuffer[0], 11);//将缓冲区数据取出
        //打印输出
        std::cout << "/* sizeof recvBuffer */" << sizeof(recvBuffer) << std::endl;
        std::cout << "/* message!!! */" << std::endl;
        std::cout << "1:  " << std::hex << static_cast<int>(receivedData.mHeader.mFrameHeader) << std::endl;
        std::cout << "2:  " << static_cast<int>(receivedData.mHeader.mFrameType) << std::endl;
        std::cout << "3:  " << std::dec << static_cast<int>(receivedData.mHeader.mFrameLength) << std::endl;
        std::cout << "4:  " << static_cast<int>(receivedData.carDriveState) << std::endl;
        std::cout << "5:  " << static_cast<int>(receivedData.carSpeed) << std::endl;
        std::cout << "6:  " << static_cast<int>(receivedData.batteryVoltage) << std::endl;
        std::cout << "7:  " << static_cast<int>(receivedData.batterySoc) << std::endl;
        std::cout << "8:  " << static_cast<int>(receivedData.carDriveMode) << std::endl;
        std::cout << "9:  " << static_cast<int>(receivedData.batteryCircuit) << std::endl;
    }
}

int main() {
    int sendSockfd, receiveSockfd;
    struct sockaddr_in serverAddr, receiveAddr, serverAddress1;

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
        perror("Error in binding for sending");
        exit(1);
    }

    std::thread sender(sendThread, sendSockfd, serverAddr);

    std::thread receiver(receiveThread, sendSockfd);

    sender.join();
    receiver.join();

    close(sendSockfd);
    close(receiveSockfd);

    return 0;
}
