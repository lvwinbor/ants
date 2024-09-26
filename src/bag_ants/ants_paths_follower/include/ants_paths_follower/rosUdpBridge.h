#ifndef ROSUDPBRIDGE_H
#define ROSUDPBRIDGE_H
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
//TODO:修改launch文件
//TODO:函数形参加const与&

// 创建UDP socket用于发送数据
class Udp
{
public:
    Udp(const int send_port,
        const int receive_port,
        const std::string &server_ip,
        const std::string &client_ip);
    void receiveData(uint8_t recv_buffer[], size_t buffer_size);
    void sendData(uint8_t send_buffer[], size_t buffer_size);

private:                                       // 变量
    const int send_port_{0};                   // 发送数据的UDP端口
    const int receive_port_{0};                // 接收数据的UDP端口
    const std::string server_ip_{"127.0.0.1"}; // 目标服务器IP地址
    const std::string client_ip_{"127.0.0.1"}; // 自身服务器IP地址
    int send_sockfd_{0};
    int receive_sockfd_{0}; // 套接字文件描述符
    struct sockaddr_in server_addr_;
    struct sockaddr_in receive_addr_;
    struct sockaddr_in client_addr_; // 服务器地址

private:                    // 函数
    void creatUdpSocket_(); // 创建UDP socket
};

#pragma pack(1)
// 自主计算机向底盘发送的操控指令
struct ComputerSendCommand
{
    uint16_t m_frame_header{0xffcc}; // 帧头
    int16_t m_frame_type{0x11};      // 帧类型
    uint8_t m_frame_length{17};      // 帧长
    int16_t vel;                     // 速度
    int16_t cur{0};                  // 曲率
    int16_t pitch{0};                // 俯仰角度
    int16_t front_arm_angle;         // 前摆臂角度
    int16_t rear_arm_angle;          // 后摆臂角度
    uint8_t flag{0};                 // 标志位
    uint8_t check_sum{0};            // 校验和
};
// 自主计算机接收的状态
struct ComputerReceiveCommand
{
    uint16_t m_frame_header; // 帧头
    int16_t m_frame_type;    // 帧类型
    uint8_t m_frame_length;  // 帧长
    int16_t vel;             // 当前速度
    int16_t cur;             // 当前曲率
    int16_t left_wheel_vel;  // 左轮速度
    int16_t right_wheel_vel; // 右轮速度
    int16_t front_arm_angle; // 前摆臂角度
    int16_t rear_arm_angle;  // 后摆臂角度
    uint8_t flag;            // 标志位
    int16_t reserve;         // 预留
    uint8_t check_sum;       // 校验和
};

// 操控端向底盘发送的操控指令
struct RemoteSendCommand
{
    uint16_t m_frame_header{0xffaa}; // 帧头
    uint8_t m_frame_type{0x0a};      // 帧类型
    uint8_t m_frame_length{19};      // 帧长
    u_char drive_gear{0b1111}; // 驱动档位、摆臂档位及摆臂控制
    u_char drive_mode{0b01};   // 操控模式切换
    u_char max_car_speed{20};  // 底盘限速
    u_char max_arm_speed{10};  // 摆臂限速设定
    char joystick_horizontal{0}; // 底盘操纵摇杆水平方向值（右摇杆）
    char joystick_vertical{0}; // 底盘操纵摇杆垂直方向值（右摇杆）
    u_char chassis_power{0b1010001}; // 底盘配电
    int16_t front_arm_angle{0};      // 前摆臂角度
    int16_t raer_arm_angle{0};       // 后摆臂角度
    u_char data_destruction{0};      // 数据销毁
    uint16_t reserved{0};            // 预留
    u_char check_sum{0};             // 校验和
};
// 操控端接收的状态
struct RemoteReceiveCommand
{
    uint16_t m_frame_header; // 帧头
    uint8_t m_frame_type;    // 帧类型
    uint8_t m_frame_length;  // 帧长
    uint8_t car_drive_state; // 平台机动状态
    int8_t car_speed;        // 平台速度
    uint8_t battery_voltage; // 电池电压（V）
    uint8_t battery_soc;     // 电池Soc
    uint8_t car_drive_mode;  // 底盘驾驶模式
    int16_t battery_circuit; // 电池电流
};

#pragma pack()

// 模板方法模式
class RosUdpBridge
{
protected:
    // ros相关变量
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_{"~"};
    uint8_t send_buffer_[32]; // 发送缓冲区
    uint8_t recv_buffer_[32]; // 接收缓冲区
    Udp udp_;

public:
    RosUdpBridge(const std::string &send_port,
                 const std::string &receive_port,
                 const std::string &server_ip,
                 const std::string &client_ip,
                 const std::string &enable_print_data,
                 const std::string &rate);
    virtual ~RosUdpBridge() = default;
    // 进行数据交换
    void dataExchange();

private:
    virtual void sendData_() = 0;        // 发送数据
    virtual void sendRos_() = 0;         // 发送ros数据
    virtual void receiveData_() = 0;     // 接收数据
    virtual void printData_() const = 0; // 打印接收到的数据
    const bool enable_print_data_;       // 是否打印数据
    ros::Rate rate_;                     // ros的频率
};
class Computer : public RosUdpBridge
{
private:
    double max_linear_speed_{1.0};
    double max_yaw_speed_{2.0};
    ComputerReceiveCommand received_data_; // 接收的数据
    std::shared_ptr<ComputerSendCommand> joy_command_{
        nullptr}; // 从手柄接收的指令
    std::shared_ptr<ComputerSendCommand> autonomy_command_{
        nullptr}; // 从自主接收的指令
    common_private_msgs::autonomyMessage motion_status_; // 发送的ros消息
    bool joy_control_mode_{false}; // 是否为手柄控制模式
    void sendData_() override;
    void receiveData_() override;
    void printData_() const override;
    void sendRos_() override;

    ros::Publisher motion_status_pub_{
        nh_.advertise<common_private_msgs::autonomyMessage>("motion_status",
                                                            1)}; // 发布底盘状态
    ros::Subscriber autonomy_sub_{
        nh_.subscribe<common_private_msgs::autonomyMessage>(
            "autonomy_command",
            1,
            &Computer::autonomyCallback_,
            this)}; // 接收自主消息
    ros::Subscriber joy_sub_{nh_.subscribe<common_private_msgs::joyMessage>(
        "joy_command", 1, &Computer::joyCallback_, this)}; // 接收手柄消息

    double speedLimit_(double speed, double max_speed);

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
    bool chassis_power_on_;              // 是否上电
    RemoteReceiveCommand received_data_; // 接收的数据
    RemoteSendCommand data_to_send_;     // 发送的数据
    void sendData_() override;
    void receiveData_() override;
    void printData_() const override;
    void sendRos_() override;
    void joyCallback_(
        const common_private_msgs::joyMessage::ConstPtr &cmd); // 手柄消息回调

    ros::Subscriber joy_sub_{nh_.subscribe<common_private_msgs::joyMessage>(
        "joy_command", 1, &Remote::joyCallback_, this)}; // 接收手柄消息

public:
    Remote();
};

#endif