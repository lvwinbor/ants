#include "rosUdpBridge.h"
#include <cstdint>
#include <memory>
#include <thread>
Udp::Udp(const int sendPort, const int receivePort, const std::string &serverIp,
         const std::string &clientIp)
    : sendPort_(sendPort),
      receivePort_(receivePort),
      serverIp_(serverIp),
      clientIp_(clientIp)
{
    creatUdpSocket_();
}
void Udp::receiveData(uint8_t recvBuffer[], size_t bufferSize)
{
    memset(recvBuffer, 0, bufferSize);        // 缓冲区初始化
    socklen_t addr_len = sizeof(clientAddr_); // 接收数据
    recvfrom(sendSockfd_, recvBuffer, bufferSize, 0,
             reinterpret_cast<struct sockaddr *>(&clientAddr_), &addr_len);
}
void Udp::sendData(uint8_t sendBuffer[], size_t bufferSize)
{
    sendto(sendSockfd_, sendBuffer, bufferSize, 0,
           reinterpret_cast<struct sockaddr *>(&serverAddr_),
           sizeof(serverAddr_));
}
void Udp::creatUdpSocket_()
{
    // 创建UDP socket用于发送数据
    sendSockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sendSockfd_ < 0)
    {
        perror("Error in socket creation for sending");
        exit(1);
    }

    // 创建UDP socket用于接收数据
    receiveSockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (receiveSockfd_ < 0)
    {
        perror("Error in socket creation for receiving");
        exit(2);
    }

    serverAddr_.sin_family = AF_INET;
    serverAddr_.sin_port = htons(sendPort_);
    serverAddr_.sin_addr.s_addr = inet_addr(serverIp_.c_str());

    receiveAddr_.sin_family = AF_INET;
    receiveAddr_.sin_port = htons(receivePort_);
    receiveAddr_.sin_addr.s_addr = inet_addr(clientIp_.c_str());

    // 感觉有问题但不建议改动
    if (bind(sendSockfd_, reinterpret_cast<struct sockaddr *>(&receiveAddr_),
             sizeof(receiveAddr_)) < 0)
    {
        perror("Vehicle: Error in binding for sending");
        exit(3);
    }
}

RosUdpBridge::RosUdpBridge(const std::string SEND_PORT,
                           const std::string RECEIVE_PORT,
                           const std::string SERVER_IP,
                           const std::string CLIENT_IP,
                           const std::string enablePrintData,
                           const std::string rate)
    : udp_(nhPrivate_.param(SEND_PORT, 0), nhPrivate_.param(RECEIVE_PORT, 0),
           nhPrivate_.param(SERVER_IP, std::string{"127.0.0.1"}),
           nhPrivate_.param(CLIENT_IP, std::string{"127.0.0.1"})),
      enablePrintData_(nhPrivate_.param(enablePrintData, false)),
      rate_(nhPrivate_.param(rate, 100)){};

void RosUdpBridge::dataExchange()
{
    while (ros::ok())
    {
        ros::spinOnce();
        sendData_();
        sendRos_();
        receiveData_();
        if (enablePrintData_)
        {
            printData_();
        }
        rate_.sleep();
    }
}
void Computer::sendData_()
{
    memset(sendBuffer_, 0, sizeof(sendBuffer_)); // 缓冲区初始化
    if (joyControlMode && joyCommand_ != nullptr)
    {
        memcpy(sendBuffer_, joyCommand_.get(), 17); // 将发送的数据存到缓冲区
    }
    else if (!joyControlMode && autonomyCommand_ != nullptr)
    {
        memcpy(sendBuffer_, autonomyCommand_.get(),
               17); // 将发送的数据存到缓冲区
    }
    else
    {
        return;
    }
    uint16_t checkSum = 0;
    // 校验和为前面数据累加和
    for (int i = 0; i < 16; i++)
    {
        checkSum += sendBuffer_[i];
    }
    sendBuffer_[16] = checkSum & 0xff; // 只保存低八位
    udp_.sendData(sendBuffer_, 17);    // 将缓冲区数据发送出去}
}
void Computer::receiveData_()
{
    udp_.receiveData(recvBuffer_, 21);       // 将数据接收到缓冲区
    memcpy(&receivedData_, recvBuffer_, 21); // 将缓冲区数据取出
    motionStatus_.rearArmAngle = static_cast<int16_t>(receivedData_.behindArm);
    motionStatus_.frontArmAngle = static_cast<int16_t>(receivedData_.frontArm);
    motionStatus_.linearVelocity = static_cast<float>(
        receivedData_.wheelVelLeft + receivedData_.wheelVelRight);
    motionStatus_.linearVelocity = static_cast<float>(
        receivedData_.wheelVelLeft + receivedData_.wheelVelRight);
}
void Computer::printData_() const
{
    std::cout << "/* message!!! */" << std::endl;
    std::cout << "1:  " << std::hex
              << static_cast<int>(receivedData_.mFrameHeader) << std::endl;
    std::cout << "2:  " << static_cast<int>(receivedData_.mFrameType)
              << std::endl;
    std::cout << "3:  " << std::dec
              << static_cast<int>(receivedData_.mFrameLength) << std::endl;
    std::cout << "4:  " << static_cast<int>(receivedData_.vel) << std::endl;
    std::cout << "5:  " << static_cast<int>(receivedData_.cur) << std::endl;
    std::cout << "6:  " << static_cast<int>(receivedData_.wheelVelLeft)
              << std::endl;
    std::cout << "7:  " << static_cast<int>(receivedData_.wheelVelRight)
              << std::endl;
    std::cout << "8:  " << static_cast<int>(receivedData_.frontArm)
              << std::endl;
    std::cout << "9:  " << static_cast<int>(receivedData_.behindArm)
              << std::endl;
    std::cout << "10:  " << static_cast<int>(receivedData_.flag) << std::endl;
    std::cout << "11:  " << static_cast<int>(receivedData_.reserve)
              << std::endl;
}
void Computer::sendRos_() { motion_status_pub_.publish(motionStatus_); }

Computer::Computer()
    : RosUdpBridge("send_port_Vehicle", "receive_port_Vehicle",
                   "server_ip_Vehicle", "client_ip_Vehicle", "udpVehiclePrint",
                   "rosRateVehicle"),
      maxLinearVelocity{nhPrivate_.param("maxLinearVelocity", 1.0)},
      maxYawVelocity(nhPrivate_.param("maxYawVelocity", 2.0))
{
}

double Computer::velocityLimit_(float velocity, double maxVelocity)
{
    if (velocity >= -maxVelocity && velocity <= maxVelocity)
    {
        return velocity;
    }
    else if (velocity < -maxVelocity)
    {
        return -maxVelocity;
    }
    else
    {
        return maxVelocity;
    }
}
void Computer::joyCallback_(
    const common_private_msgs::joyMessage::ConstPtr &command)
{
    if (joyCommand_ == nullptr)
    {
        joyCommand_ = std::make_shared<ComputerSendCommand>();
    }
    joyControlMode = command->joyControlMode;
    if (command->linearVelocity == 0.0)
    { // 中心转向，曲率绝对值的大小决定转向的速度
        joyCommand_->vel = 0;
        joyCommand_->cur = static_cast<int16_t>(command->yawVelocity * 1000.0);
    }
    else
    { // 非中心转向，曲率为半径倒数
        joyCommand_->vel =
            static_cast<int16_t>(command->linearVelocity * 100.0);
        joyCommand_->cur = static_cast<int16_t>(
            command->yawVelocity / command->linearVelocity * 1000.0);
    }
    joyCommand_->frontArm = receivedData_.frontArm;
    joyCommand_->behindArm = receivedData_.behindArm;
}
void Computer::autonomyCallback_(
    const common_private_msgs::autonomyMessage::ConstPtr &command)
{
    if (autonomyCommand_ == nullptr)
    {
        autonomyCommand_ = std::make_shared<ComputerSendCommand>();
    }
    if (command->linearVelocity == 0.0)
    { // 中心转向，曲率绝对值的大小决定转向的速度
        autonomyCommand_->vel = 0;
        autonomyCommand_->cur = static_cast<int16_t>(
            velocityLimit_(command->yawVelocity, maxYawVelocity) * 1000.0);
    }
    else
    { // 非中心转向，曲率为半径倒数
        autonomyCommand_->vel = static_cast<int16_t>(
            velocityLimit_(command->linearVelocity, maxLinearVelocity) * 100.0);
        autonomyCommand_->cur = static_cast<int16_t>(
            velocityLimit_(command->yawVelocity, maxYawVelocity) /
            velocityLimit_(command->linearVelocity, maxLinearVelocity) *
            1000.0);
    }
    autonomyCommand_->frontArm = command->frontArmAngle;
    autonomyCommand_->behindArm = command->rearArmAngle;
}

void Remote::sendData_()
{
    if (chassisPowerOn_)
    {
        dataToSend_.chassisPower = 0b1010001; // 上电
    }
    else
    {
        dataToSend_.chassisPower = 0b0; // 下电
    }

    memset(sendBuffer_, 0, sizeof(sendBuffer_)); // 缓冲区初始化
    memcpy(sendBuffer_, &dataToSend_, 19); // 将发送的数据存到缓冲区
    uint16_t checkSum = 0;
    // 校验和为前面数据累加和
    for (int i = 0; i < 18; i++)
    {
        checkSum += sendBuffer_[i];
    }
    sendBuffer_[18] = checkSum & 0xff; // 只保存低八位
    udp_.sendData(sendBuffer_, 19);    // 将缓冲区数据发送出去}
}
void Remote::receiveData_()
{
    udp_.receiveData(recvBuffer_, 11);       // 将数据接收到缓冲区
    memcpy(&receivedData_, recvBuffer_, 11); // 将缓冲区数据取出
}
void Remote::printData_() const
{
    if (chassisPowerOn_)
    { // 如果上电则打印接收到的数据
        std::cout << "/* message!!! */" << std::endl;
        std::cout << "1:  " << std::hex
                  << static_cast<int>(receivedData_.mFrameHeader) << std::endl;
        std::cout << "2:  " << static_cast<int>(receivedData_.mFrameType)
                  << std::endl;
        std::cout << "3:  " << std::dec
                  << static_cast<int>(receivedData_.mFrameLength) << std::endl;
        std::cout << "4:  " << static_cast<int>(receivedData_.carDriveState)
                  << std::endl;
        std::cout << "5:  " << static_cast<int>(receivedData_.carSpeed)
                  << std::endl;
        std::cout << "6:  " << static_cast<int>(receivedData_.batteryVoltage)
                  << std::endl;
        std::cout << "7:  " << static_cast<int>(receivedData_.batterySoc)
                  << std::endl;
        std::cout << "8:  " << static_cast<int>(receivedData_.carDriveMode)
                  << std::endl;
        std::cout << "9:  " << static_cast<int>(receivedData_.batteryCircuit)
                  << std::endl;
    }
    else
    { // 否则打印是否下电成功
        if ((receivedData_.carDriveMode & 0b1111) == 0)
        {
            std::cout << "Power-off successfully" << std::endl;
        }
        else
        {
            std::cout << "Power-off failed" << std::endl;
            std::cout << "carDriveMode"
                      << static_cast<int>(receivedData_.carDriveMode)
                      << std::endl;
        }
    }
}

Remote::Remote()
    : RosUdpBridge("send_port_Remote", "receive_port_Remote",
                   "server_ip_Remote", "client_ip_Remote", "udpRemotePrint",
                   "rosRateRemote"),
      chassisPowerOn_(nhPrivate_.param("chassisPowerOn", true))
{
}
void Remote::sendRos_() {}
void Remote::joyCallback_(const common_private_msgs::joyMessage::ConstPtr &cmd)
{
    if (cmd->remoteControlMode)
    {
        dataToSend_.driveMode = 0b10; // remote协议
    }
    else
    {
        dataToSend_.driveMode = 0b01; // computer协议
    }
    if (cmd->armAngleToZero)
    {
        dataToSend_.driveMode |= (0b1 << 6); // 摆臂角度归零
    }
    else
    {
        dataToSend_.driveMode &= ~(0b1 << 6); // 取消归零
    }
    if (cmd->joyControlMode)
    {
        switch (cmd->frontArmAngle)
        {
            case 0:
                dataToSend_.driveGear &= ~(0b11 << 4); // 摆臂角度控制
                break;
            case 1:
                dataToSend_.driveGear &= ~(0b11 << 4); // 位清零
                dataToSend_.driveGear |= (0b01 << 4);
                break;
            case -1:
                dataToSend_.driveGear &= ~(0b11 << 4); // 位清零
                dataToSend_.driveGear |= (0b10 << 4);
                break;
        }
        switch (cmd->rearArmAngle)
        {
            case 0:
                dataToSend_.driveGear &= ~(0b11 << 6); // 摆臂角度控制
                break;
            case 1:
                dataToSend_.driveGear &= ~(0b11 << 6); // 位清零
                dataToSend_.driveGear |= (0b01 << 6);
                break;
            case -1:
                dataToSend_.driveGear &= ~(0b11 << 6); // 位清零
                dataToSend_.driveGear |= (0b10 << 6);
                break;
        }
    }
}

void runComputer()
{
    std::unique_ptr<RosUdpBridge> computer = std::make_unique<Computer>();
    computer->dataExchange();
}
void runRemote()
{
    std::unique_ptr<RosUdpBridge> remote = std::make_unique<Remote>();
    remote->dataExchange();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rosUdpBridge");
    std::thread t1(runComputer);
    std::thread t2(runRemote);

    t1.join();
    t2.join();

    return 0;
}