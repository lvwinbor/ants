#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#pragma pack(1)

struct HEADER {
    uint16_t mFrameHeader;
    uint8_t mFrameType;
    uint8_t mFrameLength;
};


struct UdpControlCommand {
    HEADER mHeader;
    u_char driveGear;
    u_char driveMode;
    u_char carMaxSpeed;
    u_char armMaxSpeed;
    char chassisControlArmHorizontal;
    char chassisControlArmVertical;
    u_char chassisPower;
    int16_t frontArmAngle;
    int16_t behindArmAngle;
    u_char dataDestruction;
    uint16_t reserved;
    u_char checkSum;
};

struct UdpRecvCommand {
    HEADER mHeader;
    uint8_t carDriveState;
    int8_t carSpeed;
    uint8_t batteryVoltage;
    uint8_t batterySoc;
    uint8_t carDriveMode;
    int16_t batteryCircuit;
    // uint8_t batteryMaxTemperature;
    // uint8_t batteryMinVoltage;
    // uint8_t batteryWarning1;
    // uint8_t batterySuperMode;
    // uint8_t batteryWarning2;
    // uint8_t electrifyCommand;
    // int8_t chassisControlArmHorizontal;
    // int8_t chassisControlArmVertical;


    //    u_char armMaxSpeed;
    //    char chassisControlArmHorizontal;
    //    char chassisControlArmVertical;
    //    u_char chassisPower;
    //    int16_t frontArmAngle;
    //    int16_t behindArmAngle;
    //    u_char dataDestruction;
    //    uint16_t reserved;
    //    u_char checkSum;
    //
    //
    //    int16_t vel;
    //    int16_t cur;
    //    int16_t wheelVelLeft;
    //    int16_t wheelVelRight;
    //    int16_t reserve;
    //    uint8_t checkSum;
};

#pragma pack()
uint8_t sendBuffer[32];
uint8_t recvBuffer[32];
// int16_t recvBuffer[32];

const int SEND_PORT = 8021;    // 发送数据的UDP端口
const int RECEIVE_PORT = 20033;// 接收数据的UDP端口
//const int RECEIVE_PORT = 40021;
const char *SERVER_IP = "192.2.2.21";// 目标服务器IP地址

// 发送数据的线程函数
void sendThread(int sockfd, /* int8_t driveGear, int8_t driveMode, int8_t carMaxSpeed, int8_t chassisControlArmHorizontal,
                                    //int8_t chassisControlArmVertical, uint8_t chassisPower, uint16_t frontArmAngle, uint16_t behindArmAngle, */
                const sockaddr_in &serverAddr) {
    // struct sockaddr_in serverAddr;
    // serverAddr.sin_family = AF_INET;
    // serverAddr.sin_port = htons(SEND_PORT);
    // serverAddr.sin_addr.s_addr = inet_addr(SERVER_IP);

    while (true) {

        UdpControlCommand dataToSend;
        dataToSend.mHeader.mFrameHeader = 0xffaa;
        dataToSend.mHeader.mFrameType = 0x0a;
        dataToSend.mHeader.mFrameLength = 19;

        // dataToSend.driveGear = 0b10100111;
        dataToSend.driveGear = 0b0101;
        dataToSend.driveMode = 0b01;
        dataToSend.carMaxSpeed = 20;
        dataToSend.armMaxSpeed = 10;
        dataToSend.chassisControlArmHorizontal = 0;
        dataToSend.chassisControlArmVertical = 0;
        dataToSend.chassisPower = 0b1010001;
        dataToSend.frontArmAngle = 0;
        dataToSend.behindArmAngle = 0;
        dataToSend.dataDestruction = 0;
        dataToSend.reserved = 0;


        // dataToSend.vel = vel;
        // dataToSend.cur = cur;
        // dataToSend.pitch = pitch;


        uint16_t checkSum = 0;

        memset(&sendBuffer[0], 0, sizeof(sendBuffer));
        memcpy(&sendBuffer[0], &dataToSend, 18);

        for (int i = 0; i < 18; i++) {
            checkSum += sendBuffer[i];
        }

        sendBuffer[18] = checkSum & 0xff;

        sendto(sockfd, &sendBuffer[0], 19, 0, (struct sockaddr *) &serverAddr, sizeof(serverAddr));

        // 延迟100毫秒
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 * 1));
    }
}

// 接收数据的线程函数
void receiveThread(int sockfd) {
    while (true) {
        struct sockaddr_in clientAddr;
        socklen_t addr_len = sizeof(clientAddr);

        memset(&recvBuffer[0], 0, sizeof(recvBuffer));
        // 接收数据
        std::cout << "/* sizeof recvBuffer */" << sizeof(recvBuffer) << std::endl;
        recvfrom(sockfd, &recvBuffer[0], 16, 0, (struct sockaddr *) &clientAddr, &addr_len);
        std::cout << "/* message!!! */" << std::endl;

        std::cout << "voltage: " << 0.25 * recvBuffer[6] << std::endl;
        //        std::cout << "0:  " << recvBuffer[0] * 1 << std::endl;
        //        std::cout << "1:  " << recvBuffer[1] * 1 << std::endl;
        //        std::cout << "2:  " << recvBuffer[2] * 1 << std::endl;
        //        std::cout << "3:  " << recvBuffer[3] * 1 << std::endl;
        //        std::cout << "4:  " << recvBuffer[4] * 1 << std::endl;
        //        std::cout << "5:  " << recvBuffer[5] * 1 << std::endl;
        //        std::cout << "6:  " << recvBuffer[6] * 1 << std::endl;
        //        std::cout << "7:  " << recvBuffer[7] * 1 << std::endl;
        //        std::cout << "8:  " << recvBuffer[8] * 1 << std::endl;
        //        std::cout << "9:  " << recvBuffer[9] * 1 << std::endl;
        //        std::cout << "10:  " << recvBuffer[10] * 1 << std::endl;
        //        std::cout << "11:  " << recvBuffer[11] * 1 << std::endl;
        //        std::cout << "12:  " << recvBuffer[12] * 1 << std::endl;
        // uint16_t checkSum = 0;
        // for (int i = 0; i < 15; i++)
        // {
        //     checkSum += recvBuffer[i];
        // }
        // checkSum=checkSum&0xff;
        // std::cout << "/* message */" <<checkSum<< std::endl;
        // if ((int)recvBuffer[15] == (int)(checkSum ))
        // {

        UdpRecvCommand receivedData;
        memcpy(&receivedData, &recvBuffer[0], 9);
        // std::cout << "1:  " << std::hex << static_cast<int>(receivedData.mHeader.mFrameHeader) << std::endl;
        // std::cout << "2:  " << static_cast<int>(receivedData.mHeader.mFrameType) << std::endl;
        // std::cout << "3:  " << std::dec << static_cast<int>(receivedData.mHeader.mFrameLength) << std::endl;
        // std::cout << "4:  " << static_cast<int>(receivedData.carDriveState) << std::endl;
        // std::cout << "5:  " << static_cast<int>(receivedData.carSpeed) << std::endl;
        // std::cout << "6:  " << static_cast<int>(receivedData.batteryVoltage) << std::endl;
        // std::cout << "7:  " << static_cast<int>(receivedData.batterySoc) << std::endl;
        // std::cout << "8:  " << static_cast<int>(receivedData.carDriveMode) << std::endl;
        // std::cout << "9:  " << static_cast<int>(receivedData.batteryCircuit) << std::endl;


        //  std::cout << "10:  " << static_cast<int>(receivedData.batteryMaxTemperature) << std::endl;
        // std::cout << "11:  " << static_cast<int>(receivedData.batteryMinVoltage) << std::endl;
        // std::cout << "12:  " << static_cast<int>(receivedData.batteryWarning1) << std::endl;
        // std::cout << "13:  " << static_cast<int>(receivedData.batterySuperMode) << std::endl;
        // std::cout << "14:  " << static_cast<int>(receivedData.batteryWarning2) << std::endl;
        // std::cout << "15:  " << static_cast<int>(receivedData.electrifyCommand) << std::endl;
        // std::cout << "16:  " << static_cast<int>(receivedData.chassisControlArmHorizontal) << std::endl;
        // std::cout << "17:  " << static_cast<int>(receivedData.chassisControlArmVertical) << std::endl;

        // }
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
    //sun20240623duankoufuyong//
    // int opt=1;
    // int ret=setsockopt(sendSockfd,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof(int));
    // if(ret==-1)
    // {
    //     printf("setsockopt");
    //     exit(1);

    // }
    // duankoufuyong

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SEND_PORT);
    serverAddr.sin_addr.s_addr = inet_addr(SERVER_IP);

    receiveAddr.sin_family = AF_INET;
    receiveAddr.sin_port = htons(RECEIVE_PORT);
    receiveAddr.sin_addr.s_addr = inet_addr("192.2.2.40");

    //绑定接收端口


    if (bind(sendSockfd, (struct sockaddr *) &receiveAddr, sizeof(receiveAddr)) < 0) {
        perror("Error in binding for sending");
        exit(1);
    }


    int16_t vel = 0x64;
    int16_t cur = 0;
    int16_t pitch = 0;

    uint8_t driveGear;
    uint8_t driveMode;
    uint8_t carMaxSpeed;
    uint8_t armMaxSpeed;
    uint8_t chassisControlArmHorizontal;
    uint8_t chassisControlArmVertical;
    uint8_t chassisPower;
    uint16_t frontArmAngle;
    uint16_t behindArmAngle;

    // std::thread sender(sendThread, sendSockfd, vel, cur, pitch, serverAddr);
    std::thread sender(sendThread, sendSockfd, /*driveGear, driveMode, carMaxSpeed, armMaxSpeed, chassisControlArmHorizontal, chassisControlArmVertical, chassisPower, frontArmAngle, behindArmAngle,*/ serverAddr);

    std::thread receiver(receiveThread, sendSockfd);

    sender.join();
    receiver.join();

    close(sendSockfd);
    close(receiveSockfd);

    return 0;
}
