#include "rosUdpBridge.h"
#include <cstdint>
#include <memory>
#include <thread>
Udp::Udp(const int send_port,
         const int receive_port,
         const std::string &server_ip,
         const std::string &client_ip)
: send_port_(send_port)
, receive_port_(receive_port)
, server_ip_(server_ip)
, client_ip_(client_ip)
{
    creatUdpSocket_();
}
void Udp::receiveData(uint8_t recv_buffer[], size_t buffer_size)
{
    memset(recv_buffer, 0, buffer_size);       // 缓冲区初始化
    socklen_t addr_len = sizeof(client_addr_); // 接收数据
    recvfrom(send_sockfd_,
             recv_buffer,
             buffer_size,
             0,
             reinterpret_cast<struct sockaddr *>(&client_addr_),
             &addr_len);
}
void Udp::sendData(uint8_t send_buffer[], size_t buffer_size)
{
    sendto(send_sockfd_,
           send_buffer,
           buffer_size,
           0,
           reinterpret_cast<struct sockaddr *>(&server_addr_),
           sizeof(server_addr_));
}
void Udp::creatUdpSocket_()
{
    // 创建UDP socket用于发送数据
    send_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (send_sockfd_ < 0)
    {
        perror("Error in socket creation for sending");
        exit(1);
    }

    // 创建UDP socket用于接收数据
    receive_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (receive_sockfd_ < 0)
    {
        perror("Error in socket creation for receiving");
        exit(2);
    }

    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(send_port_);
    server_addr_.sin_addr.s_addr = inet_addr(server_ip_.c_str());

    receive_addr_.sin_family = AF_INET;
    receive_addr_.sin_port = htons(receive_port_);
    receive_addr_.sin_addr.s_addr = inet_addr(client_ip_.c_str());

    // 感觉有问题但不建议改动
    if (bind(send_sockfd_,
             reinterpret_cast<struct sockaddr *>(&receive_addr_),
             sizeof(receive_addr_)) < 0)
    {
        perror("Vehicle: Error in binding for sending");
        exit(3);
    }
}

RosUdpBridge::RosUdpBridge(const std::string &send_port,
                           const std::string &receive_port,
                           const std::string &server_ip,
                           const std::string &client_ip,
                           const std::string &enable_print_data,
                           const std::string &rate)
: udp_(nh_private_.param(send_port, 0),
       nh_private_.param(receive_port, 0),
       nh_private_.param(server_ip, std::string{"127.0.0.1"}),
       nh_private_.param(client_ip, std::string{"127.0.0.1"}))
, enable_print_data_(nh_private_.param(enable_print_data, false))
, rate_(nh_private_.param(rate, 100)){};

void RosUdpBridge::dataExchange()
{
    while (ros::ok())
    {
        ros::spinOnce();
        sendData_();
        sendRos_();
        receiveData_();
        if (enable_print_data_)
        {
            printData_();
        }
        rate_.sleep();
    }
}
void Computer::sendData_()
{
    memset(send_buffer_, 0, sizeof(send_buffer_)); // 缓冲区初始化
    if (joy_control_mode_ && joy_command_ != nullptr)
    {
        memcpy(send_buffer_, joy_command_.get(), 17); // 将发送的数据存到缓冲区
    }
    else if (!joy_control_mode_ && autonomy_command_ != nullptr)
    {
        memcpy(send_buffer_,
               autonomy_command_.get(),
               17); // 将发送的数据存到缓冲区
    }
    else
    {
        return;
    }
    uint16_t check_sum = 0;
    // 校验和为前面数据累加和
    for (int i = 0; i < 16; i++)
    {
        check_sum += send_buffer_[i];
    }
    send_buffer_[16] = check_sum & 0xff; // 只保存低八位
    udp_.sendData(send_buffer_, 17);     // 将缓冲区数据发送出去}
}
void Computer::receiveData_()
{
    udp_.receiveData(recv_buffer_, 21);        // 将数据接收到缓冲区
    memcpy(&received_data_, recv_buffer_, 21); // 将缓冲区数据取出
    motion_status_.rear_arm_angle =
        static_cast<int16_t>(received_data_.rear_arm_angle);
    motion_status_.front_arm_angle =
        static_cast<int16_t>(received_data_.front_arm_angle);
    motion_status_.linear_speed = static_cast<float>(
        received_data_.left_wheel_vel + received_data_.right_wheel_vel);
    motion_status_.linear_speed = static_cast<float>(
        received_data_.left_wheel_vel + received_data_.right_wheel_vel);
}
void Computer::printData_() const
{
    std::cout << "/* message!!! */" << std::endl;
    std::cout << "1:  " << std::hex
              << static_cast<int>(received_data_.m_frame_header) << std::endl;
    std::cout << "2:  " << static_cast<int>(received_data_.m_frame_type)
              << std::endl;
    std::cout << "3:  " << std::dec
              << static_cast<int>(received_data_.m_frame_length) << std::endl;
    std::cout << "4:  " << static_cast<int>(received_data_.vel) << std::endl;
    std::cout << "5:  " << static_cast<int>(received_data_.cur) << std::endl;
    std::cout << "6:  " << static_cast<int>(received_data_.left_wheel_vel)
              << std::endl;
    std::cout << "7:  " << static_cast<int>(received_data_.right_wheel_vel)
              << std::endl;
    std::cout << "8:  " << static_cast<int>(received_data_.front_arm_angle)
              << std::endl;
    std::cout << "9:  " << static_cast<int>(received_data_.rear_arm_angle)
              << std::endl;
    std::cout << "10:  " << static_cast<int>(received_data_.flag) << std::endl;
    std::cout << "11:  " << static_cast<int>(received_data_.reserve)
              << std::endl;
}
void Computer::sendRos_() { motion_status_pub_.publish(motion_status_); }

Computer::Computer()
: RosUdpBridge("send_port_vehicle",
               "receive_port_vehicle",
               "server_ip_vehicle",
               "client_ip_vehicle",
               "udp_vehicle_print",
               "ros_rate_vehicle")
, max_linear_speed_{nh_private_.param("max_linear_speed", 1.0)}
, max_yaw_speed_(nh_private_.param("max_yaw_speed", 2.0))
{
}

double Computer::speedLimit_(double speed, double max_speed)
{
    if (speed >= -max_speed && speed <= max_speed)
    {
        return speed;
    }
    else if (speed < -max_speed)
    {
        return -max_speed;
    }
    else
    {
        return max_speed;
    }
}
void Computer::joyCallback_(
    const common_private_msgs::joyMessage::ConstPtr &command)
{
    if (joy_command_ == nullptr)
    {
        joy_command_ = std::make_shared<ComputerSendCommand>();
    }
    joy_control_mode_ = command->joy_control_mode;
    if (command->linear_speed == 0.0)
    { // 中心转向，曲率绝对值的大小决定转向的速度
        joy_command_->vel = 0;
        joy_command_->cur = static_cast<int16_t>(command->yaw_speed * 1000.0);
    }
    else
    { // 非中心转向，曲率为半径倒数
        joy_command_->vel = static_cast<int16_t>(command->linear_speed * 100.0);
        joy_command_->cur = static_cast<int16_t>(
            command->yaw_speed / command->linear_speed * 1000.0);
    }
    joy_command_->front_arm_angle = received_data_.front_arm_angle;
    joy_command_->rear_arm_angle = received_data_.rear_arm_angle;
}
void Computer::autonomyCallback_(
    const common_private_msgs::autonomyMessage::ConstPtr &command)
{
    if (autonomy_command_ == nullptr)
    {
        autonomy_command_ = std::make_shared<ComputerSendCommand>();
    }
    if (command->linear_speed == 0.0)
    { // 中心转向，曲率绝对值的大小决定转向的速度
        autonomy_command_->vel = 0;
        autonomy_command_->cur = static_cast<int16_t>(
            speedLimit_(command->yaw_speed, max_yaw_speed_) * 1000.0);
    }
    else
    { // 非中心转向，曲率为半径倒数
        autonomy_command_->vel = static_cast<int16_t>(
            speedLimit_(command->linear_speed, max_linear_speed_) * 100.0);
        autonomy_command_->cur = static_cast<int16_t>(
            speedLimit_(command->yaw_speed, max_yaw_speed_) /
            speedLimit_(command->linear_speed, max_linear_speed_) * 1000.0);
    }
    autonomy_command_->front_arm_angle =
        static_cast<int16_t>(command->front_arm_angle);
    autonomy_command_->rear_arm_angle =
        static_cast<int16_t>(command->rear_arm_angle);
}

void Remote::sendData_()
{
    if (chassis_power_on_)
    {
        data_to_send_.chassis_power = 0b1010001; // 上电
    }
    else
    {
        data_to_send_.chassis_power = 0b0; // 下电
    }

    memset(send_buffer_, 0, sizeof(send_buffer_)); // 缓冲区初始化
    memcpy(send_buffer_, &data_to_send_, 19); // 将发送的数据存到缓冲区
    uint16_t check_sum = 0;
    // 校验和为前面数据累加和
    for (int i = 0; i < 18; i++)
    {
        check_sum += send_buffer_[i];
    }
    send_buffer_[18] = check_sum & 0xff; // 只保存低八位
    udp_.sendData(send_buffer_, 19);     // 将缓冲区数据发送出去}
}
void Remote::receiveData_()
{
    udp_.receiveData(recv_buffer_, 11);        // 将数据接收到缓冲区
    memcpy(&received_data_, recv_buffer_, 11); // 将缓冲区数据取出
}
void Remote::printData_() const
{
    if (chassis_power_on_)
    { // 如果上电则打印接收到的数据
        std::cout << "/* message!!! */" << std::endl;
        std::cout << "1:  " << std::hex
                  << static_cast<int>(received_data_.m_frame_header)
                  << std::endl;
        std::cout << "2:  " << static_cast<int>(received_data_.m_frame_type)
                  << std::endl;
        std::cout << "3:  " << std::dec
                  << static_cast<int>(received_data_.m_frame_length)
                  << std::endl;
        std::cout << "4:  " << static_cast<int>(received_data_.car_drive_state)
                  << std::endl;
        std::cout << "5:  " << static_cast<int>(received_data_.car_speed)
                  << std::endl;
        std::cout << "6:  " << static_cast<int>(received_data_.battery_voltage)
                  << std::endl;
        std::cout << "7:  " << static_cast<int>(received_data_.battery_soc)
                  << std::endl;
        std::cout << "8:  " << static_cast<int>(received_data_.car_drive_mode)
                  << std::endl;
        std::cout << "9:  " << static_cast<int>(received_data_.battery_circuit)
                  << std::endl;
    }
    else
    { // 否则打印是否下电成功
        if ((received_data_.car_drive_mode & 0b1111) == 0)
        {
            std::cout << "Power-off successfully" << std::endl;
        }
        else
        {
            std::cout << "Power-off failed" << std::endl;
            std::cout << "carDriveMode"
                      << static_cast<int>(received_data_.car_drive_mode)
                      << std::endl;
        }
    }
}

Remote::Remote()
: RosUdpBridge("send_port_remote",
               "receive_port_remote",
               "server_ip_remote",
               "client_ip_remote",
               "udp_remote_print",
               "ros_rate_remote")
, chassis_power_on_(nh_private_.param("chassis_power_on", true))
{
}
void Remote::sendRos_() { }
void Remote::joyCallback_(const common_private_msgs::joyMessage::ConstPtr &cmd)
{
    if (cmd->remote_control_mode)
    {
        data_to_send_.drive_mode = 0b10; // remote协议
    }
    else
    {
        data_to_send_.drive_mode = 0b01; // computer协议
    }
    if (cmd->arm_angle_to_zero)
    {
        data_to_send_.drive_mode |= (0b1 << 6); // 摆臂角度归零
    }
    else
    {
        data_to_send_.drive_mode &= ~(0b1 << 6); // 取消归零
    }
    if (cmd->joy_control_mode)
    {
        switch (cmd->front_arm_angle)
        {
        case 0:
            data_to_send_.drive_gear &= ~(0b11 << 4); // 摆臂角度控制
            break;
        case 1:
            data_to_send_.drive_gear &= ~(0b11 << 4); // 位清零
            data_to_send_.drive_gear |= (0b10 << 4);  //收拢
            break;
        case -1:
            data_to_send_.drive_gear &= ~(0b11 << 4); // 位清零
            data_to_send_.drive_gear |= (0b01 << 4);  //展开
            break;
        }
        switch (cmd->rear_arm_angle)
        {
        case 0:
            data_to_send_.drive_gear &= ~(0b11 << 6); // 摆臂角度控制
            break;
        case 1:
            data_to_send_.drive_gear &= ~(0b11 << 6); // 位清零
            data_to_send_.drive_gear |= (0b10 << 6);  //收拢
            break;
        case -1:
            data_to_send_.drive_gear &= ~(0b11 << 6); // 位清零
            data_to_send_.drive_gear |= (0b01 << 6);  //展开
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
    ros::init(argc, argv, "ros_udp_bridge");
    std::thread t1(runComputer);
    std::thread t2(runRemote);

    t1.join();
    t2.join();

    return 0;
}