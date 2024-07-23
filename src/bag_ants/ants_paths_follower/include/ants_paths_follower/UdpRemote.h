#include <arpa/inet.h>
#include <cstring>
#include <ros/ros.h>
#include <sys/socket.h>
#include <unistd.h>
#pragma pack(1)
// 传输数据的头部
struct HEADER {
    uint16_t mFrameHeader;//帧头
    uint8_t mFrameType;   //帧类型
    uint8_t mFrameLength; //帧长
};

// 操控端向底盘发送的操控指令
struct UdpControlCommand {
    HEADER mHeader;
    u_char driveGear;                //驱动档位、摆臂档位及摆臂控制
    u_char driveMode;                //操控模式切换
    u_char carMaxSpeed;              //底盘限速
    u_char armMaxSpeed;              //摆臂限速设定
    char chassisControlArmHorizontal;//底盘操纵摇杆水平方向值（右摇杆）
    char chassisControlArmVertical;  //底盘操纵摇杆垂直方向值（右摇杆）
    u_char chassisPower;             //底盘配电
    int16_t frontArmAngle;           //前摆臂角度
    int16_t behindArmAngle;          //后摆臂角度
    u_char dataDestruction;          //数据销毁
    uint16_t reserved;               //预留
    u_char checkSum;                 //校验和
};
// 操控端接收的状态
struct UdpRecvCommand {
    HEADER mHeader;
    uint8_t carDriveState; //平台机动状态
    int8_t carSpeed;       //平台速度
    uint8_t batteryVoltage;//电池电压（V）
    uint8_t batterySoc;    //电池Soc
    uint8_t carDriveMode;  //底盘驾驶模式
    int16_t batteryCircuit;//电池电流
};

#pragma pack()

//单例懒汉模式
class UdpRemote {
private://变量
    //ros相关变量
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate;

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
    static UdpRemote *_udpRemote;//这是单例对象，静态指针
    static std::mutex i_mutex;   //加锁
public:                          //变量
    const bool udpRemotePrint;   //是否打印数据


private://函数
    UdpRemote();
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

public:                             //函数
    void sendData();                //发送数据
    void receiveData();             //接收数据
    void printData() const;         //打印接收到的数据
    static UdpRemote *getInstance();//通过调用getInstance() 在类外获得实例
    static void deleteInstance();   //删除单例

    UdpRemote(const UdpRemote &s) = delete;
    UdpRemote &operator=(const UdpRemote &s) = delete;
};