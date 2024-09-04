#ifndef ROSUDPBRIDGE_H
#define ROSUDPBRIDGE_H
#include "ros/publisher.h"
#include <arpa/inet.h>
#include <common_private_msgs/autonomyMessage.h>
#include <common_private_msgs/joyMessage.h>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

// 创建UDP socket用于发送数据
class Udp
{
public:
    Udp(const int sendPort, const int receivePort, const std::string &serverIp,
        const std::string &clientIp);
    void receiveData(uint8_t recvBuffer[], size_t bufferSize);
    void sendData(uint8_t sendBuffer[], size_t bufferSize);

private:                                      // 变量
    const int sendPort_{0};                   // 发送数据的UDP端口
    const int receivePort_{0};                // 接收数据的UDP端口
    const std::string serverIp_{"127.0.0.1"}; // 目标服务器IP地址
    const std::string clientIp_{"127.0.0.1"}; // 自身服务器IP地址
    int sendSockfd_{0}, receiveSockfd_{0};    // 套接字文件描述符
    struct sockaddr_in serverAddr_, receiveAddr_, clientAddr_; // 服务器地址

private:                    // 函数
    void creatUdpSocket_(); // 创建UDP socket
};

#pragma pack(1)
// 自主计算机向底盘发送的操控指令
struct ComputerSendCommand
{
    uint16_t mFrameHeader{0xffcc}; // 帧头
    int16_t mFrameType{0x11};      // 帧类型
    uint8_t mFrameLength{17};      // 帧长
    int16_t vel{0};                // 速度
    int16_t cur{0};                // 曲率
    int16_t pitch{0};              // 俯仰角度
    int16_t frontArm{65};          // 前摆臂角度
    int16_t behindArm{65};         // 后摆臂角度
    uint8_t flag{0};               // 标志位
    uint8_t checkSum{0};           // 校验和
};
// 自主计算机接收的状态
struct ComputerReceiveCommand
{
    uint16_t mFrameHeader; // 帧头
    int16_t mFrameType;    // 帧类型
    uint8_t mFrameLength;  // 帧长
    int16_t vel;           // 当前速度
    int16_t cur;           // 当前曲率
    int16_t wheelVelLeft;  // 左轮速度
    int16_t wheelVelRight; // 右轮速度
    int16_t frontArm;      // 前摆臂角度
    int16_t behindArm;     // 后摆臂角度
    uint8_t flag;          // 标志位
    int16_t reserve;       // 预留
    uint8_t checkSum;      // 校验和
};

// 操控端向底盘发送的操控指令
struct RemoteSendCommand
{
    uint16_t mFrameHeader{0xffaa}; // 帧头
    uint8_t mFrameType{0x0a};      // 帧类型
    uint8_t mFrameLength{19};      // 帧长
    u_char driveGear{0b1111};      // 驱动档位、摆臂档位及摆臂控制
    u_char driveMode{0b01};        // 操控模式切换
    u_char carMaxSpeed{20};        // 底盘限速
    u_char armMaxSpeed{10};        // 摆臂限速设定
    char chassisControlArmHorizontal{0}; // 底盘操纵摇杆水平方向值（右摇杆）
    char chassisControlArmVertical{0}; // 底盘操纵摇杆垂直方向值（右摇杆）
    u_char chassisPower{0b1010001}; // 底盘配电
    int16_t frontArmAngle{0};       // 前摆臂角度
    int16_t behindArmAngle{0};      // 后摆臂角度
    u_char dataDestruction{0};      // 数据销毁
    uint16_t reserved{0};           // 预留
    u_char checkSum{0};             // 校验和
};
// 操控端接收的状态
struct RemoteReceiveCommand
{
    uint16_t mFrameHeader;  // 帧头
    uint8_t mFrameType;     // 帧类型
    uint8_t mFrameLength;   // 帧长
    uint8_t carDriveState;  // 平台机动状态
    int8_t carSpeed;        // 平台速度
    uint8_t batteryVoltage; // 电池电压（V）
    uint8_t batterySoc;     // 电池Soc
    uint8_t carDriveMode;   // 底盘驾驶模式
    int16_t batteryCircuit; // 电池电流
};

#pragma pack()

// 模板方法模式
class RosUdpBridge
{
protected:
    // ros相关变量
    ros::NodeHandle nh_;
    ros::NodeHandle nhPrivate_{"~"};
    uint8_t sendBuffer_[32]; // 发送缓冲区
    uint8_t recvBuffer_[32]; // 接收缓冲区
    Udp udp_;

public:
    RosUdpBridge(const std::string SEND_PORT, const std::string RECEIVE_PORT,
                 const std::string SERVER_IP, const std::string CLIENT_IP,
                 std::string enablePrintData, std::string rate);
    virtual ~RosUdpBridge() = default;
    // 进行数据交换
    void dataExchange();

private:
    virtual void sendData_() = 0;        // 发送数据
    virtual void sendRos_() = 0;         // 发送ros数据
    virtual void receiveData_() = 0;     // 接收数据
    virtual void printData_() const = 0; // 打印接收到的数据
    const bool enablePrintData_;         // 是否打印数据
    ros::Rate rate_;                     // ros的频率
};
class Computer : public RosUdpBridge
{
private:
    double maxLinearVelocity{1.0};
    double maxYawVelocity{2.0};
    ComputerReceiveCommand receivedData_; // 接收的数据
    std::shared_ptr<ComputerSendCommand> joyCommand_{
        nullptr}; // 从手柄接收的指令
    std::shared_ptr<ComputerSendCommand> autonomyCommand_{
        nullptr}; // 从自主接收的指令
    common_private_msgs::autonomyMessage motionStatus_; // 发送的ros消息
    bool joyControlMode{false}; // 是否为手柄控制模式
    void sendData_() override;
    void receiveData_() override;
    void printData_() const override;
    void sendRos_() override;

    ros::Publisher motion_status_pub_{
        nh_.advertise<common_private_msgs::autonomyMessage>("motionStatus",
                                                            1)}; // 发布底盘状态
    ros::Subscriber autonomy_sub_{
        nh_.subscribe<common_private_msgs::autonomyMessage>(
            "autonomyCommand", 1, &Computer::autonomyCallback_,
            this)}; // 接收自主消息
    ros::Subscriber joy_sub_{nh_.subscribe<common_private_msgs::joyMessage>(
        "joyCommand", 1, &Computer::joyCallback_, this)}; // 接收手柄消息
    bool isAutonomyInitialized_{false};                   // 是否已初始化

    double velocityLimit_(float velocity, double maxVelocity);

    void joyCallback_(
        const common_private_msgs::joyMessage::ConstPtr &cmd); // 手柄消息回调
    void autonomyCallback_(const common_private_msgs::autonomyMessage::ConstPtr
                               &cmd); // 自主消息回调

public:
    Computer();
};
class Remote : public RosUdpBridge
{
private:
    bool chassisPowerOn_;               // 是否上电
    RemoteReceiveCommand receivedData_; // 接收的数据
    RemoteSendCommand dataToSend_;      // 发送的数据
    void sendData_() override;
    void receiveData_() override;
    void printData_() const override;
    void sendRos_() override;
    void joyCallback_(
        const common_private_msgs::joyMessage::ConstPtr &cmd); // 手柄消息回调

    ros::Subscriber joy_sub_{nh_.subscribe<common_private_msgs::joyMessage>(
        "joyCommand", 1, &Remote::joyCallback_, this)}; // 接收手柄消息

public:
    Remote();
};

#endif