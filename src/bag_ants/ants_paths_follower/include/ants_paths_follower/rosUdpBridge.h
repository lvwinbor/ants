#ifndef ROSUDPBRIDGE_H
#define ROSUDPBRIDGE_H

#include "ros/init.h"
#include "ros/node_handle.h"
#include <arpa/inet.h>
#include <common_private_msgs/controlMessage.h>
#include <cstdint>
#include <cstring>
#include <ros/ros.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

// 创建UDP socket用于发送数据
class Udp {
public:
    Udp(const int sendPort, const int receivePort,
        const std::string &serverIp, const std::string &clientIp);
    void receiveData(uint8_t recvBuffer[], size_t bufferSize);
    void sendData(uint8_t sendBuffer[], size_t bufferSize);


private:                                                   //变量
    const int m_sendPort;                                  // 发送数据的UDP端口
    const int m_receivePort;                               // 接收数据的UDP端口
    const std::string m_serverIp;                          // 目标服务器IP地址
    const std::string m_clientIp;                          // 自身服务器IP地址
    int sendSockfd, receiveSockfd;                         //套接字文件描述符
    struct sockaddr_in serverAddr, receiveAddr, clientAddr;//服务器地址

private:                  //函数
    void creatUdpSocket();//创建UDP socket
};


#pragma pack(1)
// 自主计算机向底盘发送的操控指令
struct ComputerSendCommand {
    uint16_t mFrameHeader = 0xffcc;//帧头
    int16_t mFrameType = 0x11;     //帧类型
    uint8_t mFrameLength = 17;     //帧长
    int16_t vel;                   //速度
    int16_t cur;                   //曲率
    int16_t pitch = 0;             //俯仰角度
    int16_t frontArm;              //前摆臂角度
    int16_t behindArm;             //后摆臂角度
    uint8_t flag = 0;              //标志位
    uint8_t checkSum;              //校验和
};
// 自主计算机接收的状态
struct ComputerReceiveCommand {
    uint16_t mFrameHeader;//帧头
    int16_t mFrameType;   //帧类型
    uint8_t mFrameLength; //帧长
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


// 操控端向底盘发送的操控指令
struct RemoteSendCommand {
    uint16_t mFrameHeader = 0xffaa;      //帧头
    uint8_t mFrameType = 0x0a;           //帧类型
    uint8_t mFrameLength = 19;           //帧长
    u_char driveGear = 0b1010;           //驱动档位、摆臂档位及摆臂控制
    u_char driveMode = 0b01;             //操控模式切换
    u_char carMaxSpeed = 20;             //底盘限速
    u_char armMaxSpeed = 10;             //摆臂限速设定
    char chassisControlArmHorizontal = 0;//底盘操纵摇杆水平方向值（右摇杆）
    char chassisControlArmVertical = 0;  //底盘操纵摇杆垂直方向值（右摇杆）
    u_char chassisPower;                 //底盘配电
    int16_t frontArmAngle = 0;           //前摆臂角度
    int16_t behindArmAngle = 0;          //后摆臂角度
    u_char dataDestruction = 0;          //数据销毁
    uint16_t reserved = 0;               //预留
    u_char checkSum = 0;                 //校验和
};
// 操控端接收的状态
struct RemoteReceiveCommand {
    uint16_t mFrameHeader; //帧头
    uint8_t mFrameType;    //帧类型
    uint8_t mFrameLength;  //帧长
    uint8_t carDriveState; //平台机动状态
    int8_t carSpeed;       //平台速度
    uint8_t batteryVoltage;//电池电压（V）
    uint8_t batterySoc;    //电池Soc
    uint8_t carDriveMode;  //底盘驾驶模式
    int16_t batteryCircuit;//电池电流
};

#pragma pack()

const ros::NodeHandle nhPrivate("~");

//模板方法模式
class RosUdpBridge {
protected:
    //ros相关变量
    ros::NodeHandle nh;
    uint8_t sendBuffer[32];//发送缓冲区
    uint8_t recvBuffer[32];//接收缓冲区
    Udp udp;

public:
    RosUdpBridge(const int SEND_PORT, const int RECEIVE_PORT, const std::string SERVER_IP, const std::string CLIENT_IP,
                 bool enablePrintData, const int rate);
    virtual ~RosUdpBridge();
    //进行数据交换
    void dataExchange() {
        while (ros::ok()) {
            ros::spinOnce();
            sendData();
            receiveData();
            if (m_enablePrintData) {
                printData();
            }
            m_rate.sleep();
        }
    }


private:
    virtual void sendData() = 0;       //发送数据
    virtual void receiveData() = 0;    //接收数据
    virtual void printData() const = 0;//打印接收到的数据
    const bool m_enablePrintData;      //是否打印数据
    ros::Rate m_rate;                  //ros的频率
};
class Computer : public RosUdpBridge {
private:
    ComputerReceiveCommand receivedData;//接收的数据
    ComputerSendCommand dataToSend;     //发送的数据
    void sendData() override;
    void receiveData() override;
    void printData() const override;

    ros::Subscriber cmd_sub_;   //接收自主消息
    ros::Subscriber cmd_sub_joy;//接收手柄消息

    void callBack(const common_private_msgs::controlMessage::ConstPtr &command);    //回调函数
    void joyCallback(const common_private_msgs::controlMessage::ConstPtr &cmd);     //手柄消息回调
    void autonomyCallback(const common_private_msgs::controlMessage::ConstPtr &cmd);//自主消息回调

public:
    Computer();
};
class Remote : public RosUdpBridge {
private:
    bool chassisPowerOn;              //是否上电
    RemoteReceiveCommand receivedData;//接收的数据
    RemoteSendCommand dataToSend;     //发送的数据
    void sendData() override;
    void receiveData() override;
    void printData() const override;


public:
    Remote();
};

#endif