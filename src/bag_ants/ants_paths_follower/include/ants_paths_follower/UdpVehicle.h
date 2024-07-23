#include <arpa/inet.h>
#include <common_private_msgs/controlMessage.h>
#include <cstdint>
#include <cstring>
#include <geometry_msgs/TwistStamped.h>
#include <mutex>
#include <ros/ros.h>
#include <sys/socket.h>
#include <unistd.h>

#pragma pack(1)
// 传输数据的头部
struct HEADER {
    uint16_t mFrameHeader;//帧头
    int16_t mFrameType;   //帧类型
    uint8_t mFrameLength; //帧长
};
// 自主计算机向底盘发送的操控指令
struct UdpControlCommand {
    HEADER mHeader;
    int16_t vel;      //速度
    int16_t cur;      //曲率
    int16_t pitch;    //俯仰角度
    int16_t frontArm; //前摆臂角度
    int16_t behindArm;//后摆臂角度
    uint8_t flag;     //标志位
    uint8_t checkSum; //校验和
};
// 自主计算机接收的状态
struct UdpRecvCommand {
    HEADER mHeader;
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
#pragma pack()

//单例懒汉模式
class UdpVehicle {
private://变量
    //ros相关变量
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate;
    ros::Subscriber cmd_sub_;   //接收自主消息
    ros::Subscriber cmd_sub_joy;//接收手柄消息

    //udp相关变量
    UdpRecvCommand receivedData;                           //接收的数据
    UdpControlCommand dataToSend;                          //发送的数据
    uint8_t sendBuffer[32];                                //发送缓冲区
    uint8_t recvBuffer[32];                                //接收缓冲区
    const int SEND_PORT;                                   // 发送数据的UDP端口
    const int RECEIVE_PORT;                                // 接收数据的UDP端口
    const std::string SERVER_IP;                           // 目标服务器IP地址
    const std::string CLIENT_IP;                           // 自身服务器IP地址
    int sendSockfd, receiveSockfd;                         //套接字文件描述符
    struct sockaddr_in serverAddr, receiveAddr, clientAddr;//服务器地址

    //单例模式相关变量
    static UdpVehicle *_udpVehicle;//这是单例对象，静态指针
    static std::mutex i_mutex;     //加锁
public:                            //变量
    const bool udpVehiclePrint;    //是否打印数据


private://函数
    UdpVehicle();
    void callBack(const common_private_msgs::controlMessage::ConstPtr &command);
    void joyCallback(const common_private_msgs::controlMessage::ConstPtr &cmd);
    void autonomyCallback(const common_private_msgs::controlMessage::ConstPtr &cmd);
    void creatUdpSocket();//创建udp连接
    // ros::NodeHandle类没有输入名字便能直接返回值的方法，const参数的列表初始化很不方便
    template<typename T>
    T getConstParam(const std::string &paramName, ros::NodeHandle &nh) {
        T paramValue;
        if (nh.getParam(paramName, paramValue)) {
            return paramValue;
        } else {
            std::cerr << "Failed to get param" << paramName << std::endl;
            exit(4);
        }
    }

public:                              //函数
    void receiveData();              //接收数据
    void printData() const;          //打印接收到的数据
    static UdpVehicle *getInstance();//通过调用getInstance() 在类外获得实例
    static void deleteInstance();    //删除单例

    UdpVehicle(const UdpVehicle &s) = delete;
    UdpVehicle &operator=(const UdpVehicle &s) = delete;
};