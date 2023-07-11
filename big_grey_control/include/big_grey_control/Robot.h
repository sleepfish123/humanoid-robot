#include<ros/ros.h>
#include<iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include <serial/serial.h>
#include <algorithm>
#include <sensor_msgs/JointState.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;

// #define DEBUGOUTPUT //调试信息输出
#define FOOT_SENSOR_ERROR -8090
#define TimeDelay 100*1000
#define Port 4001
#define ServerIP "192.168.0.101"
// 插串口的顺序：臂、爪、腰、足
#define ARM_SERIAL_DEFAULT_NAME "/dev/ttyUSB0"    //机械臂默认串口，是1      /dev/ttyUSB0是雷达
#define ARM_SERIAL_DEFAULT_BAUDRATE 115200          //机械臂默认波特率
#define CLAW_SERIAL_DEFAULT_NAME "/dev/ttyUSB2" //默认手爪串口
#define CLAW_SERIAL_DEFAULT_BAUDRATE 9600             //手爪默认波特率
#define WAIST_SENSOR_SERIAL_DEFAULT_NAME "/dev/ttyUSB8888888" //默认腰部传感器串口2
#define WAIST_SENSOR_SERIAL_DEFAULT_BAUDRATE 9600             //腰部传感器默认波特率
#define FOOT_SENSOR_SERIAL_DEFAULT_NAME "/dev/ttyUSB3" //默认足部传感器串口3
#define FOOT_SENSOR_SERIAL_DEFAULT_BAUDRATE 9600             //足部传感器默认波特率
#define CAR_SERIAL_DEFAULT_NAME "/dev/ttyUSB0" //默认足部传感器串口3

#define FOOT_FORWARD_ROTATION 0 //脚部电机正转
#define FOOT_REVERSE_ROTATION 1   //脚步电机反转
#define WAIST_STOP 0//腰部停止
#define WAIST_DOWN 1 //腰部下降
#define WAIST_UP 2  //腰部上升

#define YXB_ID 2 //右小臂ID
#define YZB_ID 3//右肘部ID
#define YDB_ID 4//右大臂ID
#define YJCT_ID 5//右肩侧抬ID
#define YJQH_ID 6//右肩前后ID
#define DT_ID 7//点头ID
#define YT_ID 8//摇头ID
#define ZJQH_ID 9//左肩前后ID
#define ZJCT_ID 10//左肩侧抬ID
#define ZDB_ID 11//左大臂ID
#define ZZB_ID 12//左肘部ID
#define ZXB_ID 13 //左小臂ID
#define ZSW_ID 14//左手腕ID
#define ZB_ID 18//足部ID

#define WAIST1_ID 15//腰部连杆1 ID
#define WAIST2_ID 16//腰部连杆2 ID
#define WAIST3_ID 17//腰部连杆3 ID

// #define YXB_INIT 1500 //右小臂初始值
#define YXB_INIT 1000 //右小臂初始值（为了让爪子不碰杆）
#define YZB_INIT 2280//右肘部初始值
#define YDB_INIT 2522//右大臂初始值
#define YJCT_INIT 2861//右肩侧抬初始值
#define YJQH_INIT 2100//右肩前后初始值
#define DT_INIT 800//点头初始值
#define YT_INIT 520//摇头初始值
#define ZJQH_INIT 2100//左肩前后初始值
#define ZJCT_INIT 1420//左肩侧抬初始值
#define ZDB_INIT 1763//左大臂初始值
#define ZZB_INIT 2350//左肘部初始值
#define ZXB_INIT 300 //左小臂初始值
#define ZSW_INIT 680//左手腕初始值

#define YXB_SOLVE 11.4 //右小臂解算系数
#define YZB_SOLVE 12.0//右肘部解算系数
#define YDB_SOLVE 11.45//右大臂解算系数
#define YJCT_SOLVE 11.88//右肩侧抬解算系数
#define YJQH_SOLVE 11.66//右肩前后解算系数
#define DT_SOLVE 3.36//点头解算系数
#define YT_SOLVE 4.77//摇头解算系数
#define ZJQH_SOLVE 11.8//左肩前后解算系数
#define ZJCT_SOLVE 12.0//左肩侧抬解算系数
#define ZDB_SOLVE 11.45//左大臂解算系数
#define ZZB_SOLVE 12.0//左肘部解算系数
#define ZXB_SOLVE 11.0 //左小臂解算系数
#define ZSW_SOLVE 3.4//左手腕解算系数

//全局函数

//通过数据流打印输出sensor_msgs::JointState的名称与位置信息
// ostream & operator<<(ostream &cout,sensor_msgs::JointState &joint_state);

extern uint8_t arm_control_YXB[12];
extern uint8_t arm_control_YZB[12];
extern uint8_t arm_control_YDB[12];
extern uint8_t arm_control_YJCT[12];
extern uint8_t arm_control_YJQH[12];
extern uint8_t arm_control_DT[12];
extern uint8_t arm_control_YT[12];
extern uint8_t arm_control_ZXB[12];
extern uint8_t arm_control_ZDB[12];
extern uint8_t arm_control_ZZB[12];
extern uint8_t arm_control_ZJCT[12];
extern uint8_t arm_control_ZJQH[12];
extern uint8_t arm_control_ZSW[12];
extern uint8_t asynchronous_action[6];
extern uint8_t waist_sensor_recv_data[21];
extern uint8_t foot_sensor_recv_data[37];