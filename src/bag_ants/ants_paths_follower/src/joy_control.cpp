#include <common_private_msgs/joyMessage.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

bool toggleMode(bool &mode, const int button_input, bool &last_button_state)
{
    // 获取当前按键状态
    bool current_button_state = static_cast<bool>(button_input);
    // 按键从按下变为松开时切换模式
    if (!current_button_state && last_button_state)
    {
        mode = !mode;
    }
    // 更新上一个按键状态
    last_button_state = current_button_state;
    // 返回控制模式
    return mode;
}

common_private_msgs::joyMessage joy_command;
const int max_speed = 1;
const int max_angular = 2;
void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy)
{
    //axes数据为float
    //buttons数据为int

    joy_command.linear_speed = joy->axes[1] * max_speed;
    joy_command.yaw_speed = joy->axes[2] * max_angular;
    static bool joy_control_mode = false;
    static bool joy_control_mode_last = false;
    static bool remote_control_mode = false;
    static bool remote_control_mode_aast = false;
    static bool arm_angle_to_zero = false;
    static bool arm_angle_to_zero_last = false;
    if (joy->buttons[4] == 0 && joy->buttons[0] == 0)
    {
        joy_command.front_arm_angle = 0;
    }
    else if (joy->buttons[4] == 1)
    {
        joy_command.front_arm_angle = 1;
    }
    else
    {
        joy_command.front_arm_angle = -1;
    }
    if (joy->buttons[3] == 0 && joy->buttons[1] == 0)
    {
        joy_command.rear_arm_angle = 0;
    }
    else if (joy->buttons[3] == 1)
    {
        joy_command.rear_arm_angle = 1;
    }
    else
    {
        joy_command.rear_arm_angle = -1;
    }

    joy_command.joy_control_mode =
        toggleMode(joy_control_mode, joy->buttons[6], joy_control_mode_last);
    joy_command.remote_control_mode = toggleMode(
        remote_control_mode, joy->buttons[7], remote_control_mode_aast);
    joy_command.arm_angle_to_zero =
        toggleMode(arm_angle_to_zero, joy->buttons[11], arm_angle_to_zero_last);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "joy_control");
    ros::NodeHandle nh;
    ros::Subscriber sub_joy_stick =
        nh.subscribe<sensor_msgs::Joy>("joy", 1, &joystickHandler);
    ros::Publisher pub_joy_stick =
        nh.advertise<common_private_msgs::joyMessage>("joy_command", 1);
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        pub_joy_stick.publish(joy_command);
        rate.sleep();
    }
}
