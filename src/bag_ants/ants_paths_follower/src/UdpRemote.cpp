#include "UdpRemote.h"
#include <iostream>

UdpRemote *UdpRemote::getInstance() {
    if (_udpRemote == nullptr) {//先判断是否为空，如果为空则进入，不为空说明已经存在实例，直接返回
        i_mutex.lock();
        if (_udpRemote == nullptr) {//再判断一次，确保不会因为加锁期间多个线程同时进入
            _udpRemote = new UdpRemote();
        }
        i_mutex.unlock();
    }
    return _udpRemote;//返回这个实例的地址
}
void UdpRemote::deleteInstance() {
    if (_udpRemote != nullptr) {
        i_mutex.lock();
        if (_udpRemote != nullptr) {
            delete _udpRemote;
            _udpRemote = nullptr;
        }
        i_mutex.unlock();
    }
}
UdpRemote::UdpRemote() : nhPrivate("~"),
                         SEND_PORT(getConstParam<int>("send_port_Remote", nhPrivate)),
                         RECEIVE_PORT(getConstParam<int>("receive_port_Remote", nhPrivate)),
                         SERVER_IP(getConstParam<std::string>("server_ip_Remote", nhPrivate)),
                         CLIENT_IP(getConstParam<std::string>("client_ip_Remote", nhPrivate)),
                         udpRemotePrint(getConstParam<bool>("udpRemotePrint", nhPrivate)) {


    creatUdpSocket();
}

void UdpRemote::sendData() {

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

    memset(&sendBuffer[0], 0, sizeof(sendBuffer));//缓冲区初始化
    memcpy(&sendBuffer[0], &dataToSend, 19);      //将发送的数据存到缓冲区
    // 校验和为前面数据累加和
    uint16_t checkSum = 0;
    for (int i = 0; i < 18; i++) {
        checkSum += sendBuffer[i];
    }


    sendBuffer[18] = checkSum & 0xff;//只保存低八位
    //发送数据
    sendto(sendSockfd, &sendBuffer[0], 19, 0, reinterpret_cast<const struct sockaddr *>(&serverAddr), sizeof(serverAddr));
}

//接收数据的函数
void UdpRemote::receiveData() {
    memset(&recvBuffer[0], 0, sizeof(recvBuffer));//缓冲区初始化
    // 接收数据
    socklen_t addr_len = sizeof(clientAddr);
    recvfrom(sendSockfd, &recvBuffer[0], 11, 0, reinterpret_cast<struct sockaddr *>(&clientAddr), &addr_len);
    memcpy(&receivedData, &recvBuffer[0], 11);//将缓冲区数据取出
}
void UdpRemote::printData() const {
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
void UdpRemote::creatUdpSocket() {
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
    serverAddr.sin_addr.s_addr = inet_addr(SERVER_IP.c_str());

    receiveAddr.sin_family = AF_INET;
    receiveAddr.sin_port = htons(RECEIVE_PORT);
    receiveAddr.sin_addr.s_addr = inet_addr(CLIENT_IP.c_str());

    //感觉有问题但不建议改动
    if (bind(sendSockfd, reinterpret_cast<struct sockaddr *>(&receiveAddr), sizeof(receiveAddr)) < 0) {
        perror("Remote: Error in binding for receiving");
        exit(1);
    }
}
//静态成员变量类外初始化
UdpRemote *UdpRemote::_udpRemote = nullptr;
std::mutex UdpRemote::i_mutex;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "udp_remote");
    UdpRemote *udpRemote = UdpRemote::getInstance();
    // ros::Publisher feedBack = nh.advertise<geometry_msgs::TwistStamped>("feedback_vel", 5);//将速度发布到cmd_vel中


    ros::Rate rate(100);
    if (udpRemote->udpRemotePrint) {
        while (ros::ok()) {
            udpRemote->sendData();
            udpRemote->receiveData();
            udpRemote->printData();
            rate.sleep();
        }
    } else {
        while (ros::ok()) {
            udpRemote->sendData();
            udpRemote->receiveData();
            rate.sleep();
        }
    }
    UdpRemote::deleteInstance();
    return 0;
}
