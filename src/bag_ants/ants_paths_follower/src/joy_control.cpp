#include <common_private_msgs/joyMessage.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


bool toggleMode(bool &mode, const int buttonInput, bool &lastButtonState) {
    // 获取当前按键状态
    bool currentButtonState = static_cast<bool>(buttonInput);
    // 按键从按下变为松开时切换模式
    if (!currentButtonState && lastButtonState) {
        mode = !mode;
    }
    // 更新上一个按键状态
    lastButtonState = currentButtonState;
    // 返回控制模式
    return mode;
}

common_private_msgs::joyMessage joyCommand;
const int maxSpeed = 1;
const int maxAngular = 2;
void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy) {
    //axes数据为float
    //buttons数据为int

    joyCommand.linearVelocity = joy->axes[1] * maxSpeed;
    joyCommand.yawVelocity = joy->axes[2] * maxAngular;
    static bool joyControlMode = false;
    static bool joyControlModeLast = false;
    static bool remoteControlMode = false;
    static bool remoteControlModeLast = false;
    static bool armAngleToZero = false;
    static bool armAngleToZeroLast = false;
    if (joy->buttons[4] == 0 && joy->buttons[0] == 0) {
        joyCommand.frontArmAngle = 0;
    } else if (joy->buttons[4] == 1) {
        joyCommand.frontArmAngle = 1;
    } else {
        joyCommand.frontArmAngle = -1;
    }
    if (joy->buttons[3] == 0 && joy->buttons[1] == 0) {
        joyCommand.rearArmAngle = 0;
    } else if (joy->buttons[3] == 1) {
        joyCommand.rearArmAngle = 1;
    } else {
        joyCommand.rearArmAngle = -1;
    }

    joyCommand.joyControlMode = toggleMode(joyControlMode, joy->buttons[6], joyControlModeLast);
    joyCommand.remoteControlMode = toggleMode(remoteControlMode, joy->buttons[7], remoteControlModeLast);
    joyCommand.armAngleToZero = toggleMode(armAngleToZero, joy->buttons[11], armAngleToZeroLast);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "joy_control");
    // ros::NodeHandle nh("~");
    ros::NodeHandle nh;
    ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("joy", 1, &joystickHandler);
    ros::Publisher pubJoystick = nh.advertise<common_private_msgs::joyMessage>("joyCommand", 1);
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        pubJoystick.publish(joyCommand);
        rate.sleep();
    }
}
