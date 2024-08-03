#include "rosUdpBridge.h"


Udp::Udp(const int sendPort, const int receivePort,
         const std::string &serverIp, const std::string &clientIp)
    : m_sendPort(sendPort),
      m_receivePort(receivePort),
      m_serverIp(serverIp),
      m_clientIp(clientIp) {

    creatUdpSocket();
}
void Udp::receiveData(uint8_t recvBuffer[], size_t bufferSize) {
    memset(recvBuffer, 0, bufferSize);      //缓冲区初始化
    socklen_t addr_len = sizeof(clientAddr);// 接收数据
    recvfrom(sendSockfd, recvBuffer, bufferSize, 0, reinterpret_cast<struct sockaddr *>(&clientAddr), &addr_len);
}
void Udp::sendData(uint8_t sendBuffer[], size_t bufferSize) {
    sendto(sendSockfd, sendBuffer, bufferSize, 0, reinterpret_cast<struct sockaddr *>(&serverAddr), sizeof(serverAddr));
}
void Udp::creatUdpSocket() {
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
    serverAddr.sin_port = htons(m_sendPort);
    serverAddr.sin_addr.s_addr = inet_addr(m_serverIp.c_str());

    receiveAddr.sin_family = AF_INET;
    receiveAddr.sin_port = htons(m_receivePort);
    receiveAddr.sin_addr.s_addr = inet_addr(m_clientIp.c_str());

    //感觉有问题但不建议改动
    if (bind(sendSockfd, reinterpret_cast<struct sockaddr *>(&receiveAddr), sizeof(receiveAddr)) < 0) {
        perror("Vehicle: Error in binding for sending");
        exit(3);
    }
}


RosUdpBridge::RosUdpBridge(const int SEND_PORT, const int RECEIVE_PORT, const std::string SERVER_IP, const std::string CLIENT_IP,
                           bool enablePrintData, const int rate)
    : udp(SEND_PORT, RECEIVE_PORT, SERVER_IP, CLIENT_IP),
      m_enablePrintData(enablePrintData), m_rate(rate){};

void Computer::sendData() {
    memset(sendBuffer, 0, sizeof(sendBuffer));          //缓冲区初始化
    memcpy(sendBuffer, &dataToSend, sizeof(sendBuffer));//将发送的数据存到缓冲区
    uint16_t checkSum = 0;
    // 校验和为前面数据累加和
    for (int i = 0; i < 16; i++) {
        checkSum += sendBuffer[i];
    }
    sendBuffer[16] = checkSum & 0xff;            //只保存低八位
    udp.sendData(sendBuffer, sizeof(sendBuffer));//将缓冲区数据发送出去}
}
void Computer::receiveData() {
    udp.receiveData(recvBuffer, sizeof(recvBuffer));      //将数据接收到缓冲区
    memcpy(&receivedData, recvBuffer, sizeof(recvBuffer));//将缓冲区数据取出
}
void Computer::printData() const {
    std::cout << "/* message!!! */" << std::endl;
    std::cout << "1:  " << std::hex << static_cast<int>(receivedData.mFrameHeader) << std::endl;
    std::cout << "2:  " << static_cast<int>(receivedData.mFrameType) << std::endl;
    std::cout << "3:  " << std::dec << static_cast<int>(receivedData.mFrameLength) << std::endl;
    std::cout << "4:  " << static_cast<int>(receivedData.vel) << std::endl;
    std::cout << "5:  " << static_cast<int>(receivedData.cur) << std::endl;
    std::cout << "6:  " << static_cast<int>(receivedData.wheelVelLeft) << std::endl;
    std::cout << "7:  " << static_cast<int>(receivedData.wheelVelRight) << std::endl;
    std::cout << "8:  " << static_cast<int>(receivedData.frontArm) << std::endl;
    std::cout << "9:  " << static_cast<int>(receivedData.behindArm) << std::endl;
    std::cout << "10:  " << static_cast<int>(receivedData.flag) << std::endl;
    std::cout << "11:  " << static_cast<int>(receivedData.reserve) << std::endl;
}

Computer::Computer() : RosUdpBridge(nhPrivate.param("send_port_Vehicle", 0),
                                    nhPrivate.param("receive_port_Vehicle", 0),
                                    nhPrivate.param("server_ip_Vehicle", std::string("127.0.0.1")),
                                    nhPrivate.param("client_ip_Vehicle", std::string("127.0.0.1")),
                                    nhPrivate.param("udpVehiclePrint", false),
                                    nhPrivate.param("rosRateVehicle", 100)) {

    cmd_sub_ = nh.subscribe<common_private_msgs::controlMessage>("autonomyCommand", 5, &Computer::autonomyCallback, this);
    cmd_sub_joy = nh.subscribe<common_private_msgs::controlMessage>("joyCommand", 1, &Computer::joyCallback, this);
}
void Computer::callBack(const common_private_msgs::controlMessage::ConstPtr &command) {

    if (command->linearVelocity == 0.0) {//中心转向，曲率绝对值的大小决定转向的速度
        dataToSend.vel = 0;
        dataToSend.cur = static_cast<int16_t>(command->yawVelocity * 1000.0);
    } else {//非中心转向，曲率为半径倒数
        dataToSend.vel = static_cast<int16_t>(command->linearVelocity * 100.0);
        dataToSend.cur = static_cast<int16_t>(command->yawVelocity / command->yawVelocity * 1000.0);
    }
    dataToSend.frontArm = command->frontArmAngle;
    dataToSend.behindArm = command->rearArmAngle;
}
void Computer::joyCallback(const common_private_msgs::controlMessage::ConstPtr &cmd) {
    if (cmd->joyControlMode) {//处于手柄控制模式
        callBack(cmd);
    }
}
void Computer::autonomyCallback(const common_private_msgs::controlMessage::ConstPtr &cmd) {
    if (!(cmd->joyControlMode)) {//处于自主模式
        callBack(cmd);
    }
}


void Remote::sendData() {
    if (chassisPowerOn) {
        dataToSend.chassisPower = 0b1010001;//上电
    } else {
        dataToSend.chassisPower = 0b0;//下电
    }

    memset(sendBuffer, 0, sizeof(sendBuffer));          //缓冲区初始化
    memcpy(sendBuffer, &dataToSend, sizeof(sendBuffer));//将发送的数据存到缓冲区
    uint16_t checkSum = 0;
    // 校验和为前面数据累加和
    for (int i = 0; i < 18; i++) {
        checkSum += sendBuffer[i];
    }
    sendBuffer[18] = checkSum & 0xff;            //只保存低八位
    udp.sendData(sendBuffer, sizeof(sendBuffer));//将缓冲区数据发送出去}
}
void Remote::receiveData() {
    udp.receiveData(recvBuffer, sizeof(recvBuffer));      //将数据接收到缓冲区
    memcpy(&receivedData, recvBuffer, sizeof(recvBuffer));//将缓冲区数据取出
}
void Remote::printData() const {
    if (chassisPowerOn) {//如果上电则打印接收到的数据
        std::cout << "/* message!!! */" << std::endl;
        std::cout << "1:  " << std::hex << static_cast<int>(receivedData.mFrameHeader) << std::endl;
        std::cout << "2:  " << static_cast<int>(receivedData.mFrameType) << std::endl;
        std::cout << "3:  " << std::dec << static_cast<int>(receivedData.mFrameLength) << std::endl;
        std::cout << "4:  " << static_cast<int>(receivedData.carDriveState) << std::endl;
        std::cout << "5:  " << static_cast<int>(receivedData.carSpeed) << std::endl;
        std::cout << "6:  " << static_cast<int>(receivedData.batteryVoltage) << std::endl;
        std::cout << "7:  " << static_cast<int>(receivedData.batterySoc) << std::endl;
        std::cout << "8:  " << static_cast<int>(receivedData.carDriveMode) << std::endl;
        std::cout << "9:  " << static_cast<int>(receivedData.batteryCircuit) << std::endl;
    } else {//否则打印是否下电成功
        if (!static_cast<bool>(receivedData.carDriveMode)) {
            std::cout << "Power-off successfully" << std::endl;
        } else {
            std::cout << "Power-off failed" << std::endl;
        }
    }
}

Remote::Remote() : RosUdpBridge(nhPrivate.param("send_port_Remote", 0),
                                nhPrivate.param("receive_port_Remote", 0),
                                nhPrivate.param("server_ip_Remote", std::string("127.0.0.1")),
                                nhPrivate.param("client_ip_Remote", std::string("127.0.0.1")),
                                nhPrivate.param("udpRemotePrint", false),
                                nhPrivate.param("rosRateRemote", 100)),
                   chassisPowerOn(nhPrivate.param("chassisPowerOn", true)) {}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "rosUdpBridge");
    RosUdpBridge *computer = new Computer;
    computer->dataExchange();
    RosUdpBridge *remote = new Remote;
    remote->dataExchange();
    delete computer;
    delete remote;
}