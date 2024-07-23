#include "UdpVehicle.h"
#include <iostream>


UdpVehicle *UdpVehicle::getInstance() {
    if (_udpVehicle == nullptr) {//先判断是否为空，如果为空则进入，不为空说明已经存在实例，直接返回
        i_mutex.lock();
        if (_udpVehicle == nullptr) {//再判断一次，确保不会因为加锁期间多个线程同时进入
            _udpVehicle = new UdpVehicle();
        }
        i_mutex.unlock();
    }
    return _udpVehicle;//返回这个实例的地址
}
void UdpVehicle::deleteInstance() {
    if (_udpVehicle != nullptr) {
        i_mutex.lock();
        if (_udpVehicle != nullptr) {
            delete _udpVehicle;
            _udpVehicle = nullptr;
        }
        i_mutex.unlock();
    }
}
UdpVehicle::UdpVehicle() : nhPrivate("~"),
                           SEND_PORT(getConstParam<int>("send_port_Vehicle", nhPrivate)),
                           RECEIVE_PORT(getConstParam<int>("receive_port_Vehicle", nhPrivate)),
                           SERVER_IP(getConstParam<std::string>("server_ip_Vehicle", nhPrivate)),
                           CLIENT_IP(getConstParam<std::string>("client_ip_Vehicle", nhPrivate)),
                           udpVehiclePrint(getConstParam<bool>("udpVehiclePrint", nhPrivate)) {

    cmd_sub_ = nh.subscribe<common_private_msgs::controlMessage>("autonomyCommand", 5, &UdpVehicle::autonomyCallback, this);
    cmd_sub_joy = nh.subscribe<common_private_msgs::controlMessage>("joyCommand", 1, &UdpVehicle::joyCallback, this);
    creatUdpSocket();
}
void UdpVehicle::callBack(const common_private_msgs::controlMessage::ConstPtr &command) {
    dataToSend.mHeader.mFrameHeader = 0xffcc;//固定值
    dataToSend.mHeader.mFrameType = 0x11;    //固定值
    dataToSend.mHeader.mFrameLength = 17;    //固定值

    if (command->linearVelocity == 0.0) {//中心转向，曲率绝对值的大小决定转向的速度
        dataToSend.vel = 0;
        dataToSend.cur = static_cast<int16_t>(command->yawVelocity * 1000.0);
    } else {//非中心转向，曲率为半径倒数
        dataToSend.vel = static_cast<int16_t>(command->linearVelocity * 100.0);
        dataToSend.cur = static_cast<int16_t>(command->yawVelocity / command->yawVelocity * 1000.0);
    }

    dataToSend.pitch = 0;//不控制俯仰角度
    dataToSend.frontArm = command->frontArmAngle;
    dataToSend.behindArm = command->rearArmAngle;
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

void UdpVehicle::joyCallback(const common_private_msgs::controlMessage::ConstPtr &cmd) {
    if (cmd->joyControlMode) {//处于手柄控制模式
        callBack(cmd);
    }
}
void UdpVehicle::autonomyCallback(const common_private_msgs::controlMessage::ConstPtr &cmd) {
    if (!(cmd->joyControlMode)) {//处于自主模式
        callBack(cmd);
    }
}

//接收数据的函数
void UdpVehicle::receiveData() {
    memset(&recvBuffer[0], 0, sizeof(recvBuffer));//缓冲区初始化
    // 接收数据
    socklen_t addr_len = sizeof(clientAddr);
    recvfrom(sendSockfd, &recvBuffer[0], 21, 0, reinterpret_cast<struct sockaddr *>(&clientAddr), &addr_len);
    memcpy(&receivedData, &recvBuffer[0], 21);//将缓冲区数据取出
}
void UdpVehicle::printData() const {
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
void UdpVehicle::creatUdpSocket() {
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
        exit(2);
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SEND_PORT);
    serverAddr.sin_addr.s_addr = inet_addr(SERVER_IP.c_str());

    receiveAddr.sin_family = AF_INET;
    receiveAddr.sin_port = htons(RECEIVE_PORT);
    receiveAddr.sin_addr.s_addr = inet_addr(CLIENT_IP.c_str());


    //感觉有问题但不建议改动
    if (bind(sendSockfd, reinterpret_cast<struct sockaddr *>(&receiveAddr), sizeof(receiveAddr)) < 0) {
        perror("Vehicle: Error in binding for sending");
        exit(3);
    }
}
//静态成员变量类外初始化
UdpVehicle *UdpVehicle::_udpVehicle = nullptr;
std::mutex UdpVehicle::i_mutex;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "udp_vehicle");
    UdpVehicle *udpVehicle = UdpVehicle::getInstance();
    // ros::Publisher feedBack = nh.advertise<geometry_msgs::TwistStamped>("feedback_vel", 5);//将速度发布到cmd_vel中


    ros::Rate rate(100);
    if (udpVehicle->udpVehiclePrint) {
        while (ros::ok()) {
            ros::spinOnce();
            udpVehicle->receiveData();
            udpVehicle->printData();
            rate.sleep();
        }
    } else {
        while (ros::ok()) {
            ros::spinOnce();
            udpVehicle->receiveData();
            rate.sleep();
        }
    }
    UdpVehicle::deleteInstance();
    return 0;
}
