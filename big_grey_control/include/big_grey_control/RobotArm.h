#pragma once
#include "big_grey_control/Robot.h"
class RobotArm
{
    // friend ostream & operator<<(ostream &cout,sensor_msgs::JointState &joint_state);//全局友元函数，可以访问私有变量
public:
    RobotArm(const char* serial_name,int baudrate);//带参构造函数
    ~RobotArm();
    void arm_msg(uint8_t *arm_control,int id,double angle);//arm_control为控制数组，是输出
    void publish();//可发布上半身所有关节角度，单位是角度
    void action_command_msg(uint8_t *action);//异步控制数组赋值函数
    void arm_control(sensor_msgs::JointState &robot_arm_states);//发送控制信息，顺序为ID顺序
    void head_control(sensor_msgs::JointState &robot_head_states);//头部控制
    void arm_track_control(sensor_msgs::JointState &robot_head_states);// 右臂追踪控制
    void arm_FBtrack_control(sensor_msgs::JointState &robot_head_states);// 右臂前后追踪控制
    void arm_init();//机器人关节初始化
    double angle2rad(double angle);//角度转化弧度
    double rad2angle(double rad);//弧度转化角度
    void joint_state_assign(sensor_msgs::JointState &joint_state);//为joint_state的关节位置赋值，顺序为ID顺序,单位是角度
    size_t control_msg_length;
    size_t action_msg_length;    
    serial::Serial ser_arm;
private:
    // serial::Serial ser_arm;
   char serial_name[13];//备份串口名称
    ros::NodeHandle nh;
    ros::Publisher robot_arm_pub;
    sensor_msgs::JointState robot_arm_states;
    bool isInit;
    double rad_YXB;
    double rad_YZB;
    double rad_YDB;
    double rad_YJCT;
    double rad_YJQH;
    double rad_DT;
    double rad_YT;
    double rad_ZXB;
    double rad_ZDB;
    double rad_ZZB;
    double rad_ZJCT;
    double rad_ZJQH;
    double rad_ZSW;

    double angle_YXB;
    double angle_YZB;
    double angle_YDB;
    double angle_YJCT;
    double angle_YJQH;
    double angle_DT;
    double angle_YT;
    double angle_ZXB;
    double angle_ZDB;
    double angle_ZZB;
    double angle_ZJCT;
    double angle_ZJQH;
    double angle_ZSW;
};
