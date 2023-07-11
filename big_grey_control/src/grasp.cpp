#include<big_grey_control/RobotArm.h>
#include "big_grey_control/servo_command.h"
#include<big_grey_control/RobotClaw.h>
#include<big_grey_control/RobotWaist.h>
#include<big_grey_control/RobotCar.h>
#include "big_grey_control/big_grey_srv.h"
#include "sensor_msgs/JointState.h"
#include<big_grey_control/Robot.h>
#include <unistd.h> // 延时函数头文件 sleep(5); // 延迟5秒
#include <iostream>

using namespace std;

serial::Serial ser; // 声明串口对象

double car_qianhou;
double car_zuoyou;
double rad_jiaobu, theta_jiaobu;
double up_down_waist;
double rad_wanyao, theta_wanyao;
double rad_ceyao, theta_ceyao;
double rad_qianhoutai, theta_qianhoutai;
double rad_cetai, theta_cetai;
double rad_dabi, theta_dabi;
double rad_zhoubu, theta_zhoubu;
double L1, L2, L3;

// 弧度转角度
double rad2angle(double rad)
{
    double angle=rad*180.0/M_PI;
    return angle;
}

int flag = 0; // 接收标志位
int flag_server = 0;

void chatterCallback(const sensor_msgs::JointState& msg)
{
    car_qianhou = msg.position[0];
    car_zuoyou = msg.position[1];
    rad_jiaobu = msg.position[2];
    up_down_waist = msg.position[3];
    rad_wanyao = msg.position[4];
    rad_ceyao = msg.position[5];
    rad_qianhoutai = msg.position[6];
    rad_cetai = msg.position[7];
    rad_dabi = msg.position[8];
    rad_zhoubu = msg.position[9];
    L1 = msg.position[10];
    L2 = msg.position[11];
    L3 = msg.position[12];
    
    if (flag == 0) {
        cout << "************grasp接收到的数据*******************" << endl;
        cout << msg.name[0] << " = " << car_qianhou << endl; // car_qianhou
        cout << msg.name[1] << " = " << car_zuoyou << endl; // car_zuoyou
        cout << msg.name[2] << " = " << rad_jiaobu << endl; // rad_jiaobu
        cout << msg.name[3] << " = " << up_down_waist << endl; // up_down_waist
        cout << msg.name[4] << " = " << rad_wanyao << endl; // rad_wanyao
        cout << msg.name[5] << " = " << rad_ceyao << endl; // rad_ceyao
        cout << msg.name[6] << " = " << rad_qianhoutai << endl; // rad_qianhoutai
        cout << msg.name[7] << " = " << rad_cetai << endl; // rad_cetai
        cout << msg.name[8] << " = " << rad_dabi << endl; // rad_dabi
        cout << msg.name[9] << " = " << rad_zhoubu << endl; // rad_zhoubu
        cout << msg.name[10] << " = " << L1 << endl; // L1
        cout << msg.name[11] << " = " << L2 << endl; // L2
        cout << msg.name[12] << " = " << L3 << endl; // L3
        flag = 1;
    }
}

bool grasp(big_grey_control::big_grey_srv::Request &req, big_grey_control::big_grey_srv::Response &res) {
    if (req.req.position[0] == 2.2) {
        ROS_INFO("grasp已响应");
        flag_server = 1;

        car_qianhou = req.req.position[1];
        car_zuoyou = req.req.position[2];
        rad_jiaobu = req.req.position[3];
        up_down_waist = req.req.position[4];
        rad_wanyao = req.req.position[5];
        rad_ceyao = req.req.position[6];
        rad_qianhoutai = req.req.position[7];
        rad_cetai = req.req.position[8];
        rad_dabi = req.req.position[9];
        rad_zhoubu = req.req.position[10];
        L1 = req.req.position[11]; // 单位m
        L2 = req.req.position[12];
        L3 = req.req.position[13];

        cout << "************GRASP接收到的数据*******************" << endl;
        cout << "car_qianhou = " << car_qianhou << endl; // car_qianhou
        cout << "car_zuoyou = " << car_zuoyou << endl; // car_zuoyou
        cout << "rad_jiaobu = " << rad_jiaobu << endl; // rad_jiaobu
        cout << "up_down_waist = " << up_down_waist << endl; // up_down_waist
        cout << "rad_wanyao = " << rad_wanyao << endl; // rad_wanyao
        cout << "rad_ceyao = " << rad_ceyao << endl; // rad_ceyao
        cout << "rad_qianhoutai = " << rad_qianhoutai << endl; // rad_qianhoutai
        cout << "rad_cetai = " << rad_cetai << endl; // rad_cetai
        cout << "rad_dabi = " << rad_dabi << endl; // rad_dabi
        cout << "rad_zhoubu = " << rad_zhoubu << endl; // rad_zhoubu
        cout << "L1 = " << L1 << endl; // L1
        cout << "L2 = " << L2 << endl; // L2
        cout << "L3 = " << L3 << endl; // L3

        // 反馈响应
        res.res.name.clear();
        res.res.name.push_back("grasp");

        res.res.position.clear();
        res.res.position.push_back(4.4);

        for(int i=0;i<res.res.name.size();i++)
        {
            cout<<"***GRASP  "<<ends;
            cout<<res.res.name[i]<<"="
            <<res.res.position[i]<<"***"<<endl;
        }
    }
    else {
        ROS_INFO("grasp no receive request");
    }
    
    return true;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,""); // 中文输出
    ros::init(argc,argv,"grasp_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("send_rad", 1000, chatterCallback); // 监听send_rad话题，接收各关节运动量的数据
    ros::ServiceServer grasp_service = n.advertiseService("grasp_server", grasp);
    ROS_INFO("Grasp Init Ready? ");

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        if (flag_server==1 && flag==1) {
            cout << "****************grasp开始控制********************" << endl;
            cout << "****************已接收到规划数据********************" << endl;
            /*********弧度值转角度值***********/
            theta_zhoubu = rad2angle(rad_zhoubu);
            theta_dabi = rad2angle(rad_dabi);
            theta_cetai = rad2angle(rad_cetai);
            theta_qianhoutai = rad2angle(rad_qianhoutai);
            /***********************************/
            sensor_msgs::JointState armControl; // 控制手臂所需参数为sensor_msgs::JointState类型
            armControl.position[0] = 0; // 右小臂（角度值）
            armControl.position[1] = theta_zhoubu; // 右肘部
            armControl.position[2] = theta_dabi; // 右大臂
            armControl.position[3] = theta_cetai; // 右肩侧抬
            armControl.position[4] = theta_qianhoutai; // 右肩前后
            armControl.position[5] = 0; // 点头
            armControl.position[6] = 0; // 摇头
            armControl.position[7] = 0; // 左肩前后
            armControl.position[8] = 0; // 左肩侧抬
            armControl.position[9] = 0; // 左大臂
            armControl.position[10] = 0; // 左肘部
            armControl.position[11] = 0; // 左小臂
            armControl.position[12] = 0; // 左手腕

            /******************************* 下发控制指令 *************************************/
            ROS_INFO("******************下发控制指令******************");
            // // 手臂
            // RobotArm RobotArm_(ARM_SERIAL_DEFAULT_NAME, ARM_SERIAL_DEFAULT_BAUDRATE);
            // // 爪子
            // RobotClaw RobotClaw_(CLAW_SERIAL_DEFAULT_NAME,CLAW_SERIAL_DEFAULT_BAUDRATE);
            // // 腰部
            // int sockfd=socket(PF_INET, SOCK_STREAM,0);
            // RobotWaist RobotWaist_(sockfd,WAIST_SENSOR_SERIAL_DEFAULT_NAME,WAIST_SENSOR_SERIAL_DEFAULT_BAUDRATE);
            // theta_jiaobu = rad2angle(rad_jiaobu); // 脚部弧度转角度

            // /*******************************去抓******************************/
            // // 先手臂
            // RobotArm_.arm_control(armControl);
            // // 再腰
            // RobotWaist_.foot_waist_action_control(theta_ceyao, theta_wanyao, up_down_waist * 1000, theta_jiaobu); // 角度、mm
            // // 再爪子
            // RobotClaw_.claw_close_control(); // 爪子闭合

            // /*******************************抓完回来******************************/
            // // 先爪子
            // RobotClaw_.claw_open_control(); // 爪子张开
            // // 再手臂
            // RobotArm_.arm_init(); // 手臂初始化
            // // 再腰
            // RobotWaist_.foot_waist_action_control(-theta_ceyao, -theta_wanyao, -up_down_waist * 1000, -theta_jiaobu); // 角度、mm
            cout << "****************结束规划********************" << endl;
            sleep(30); // ........................
            ros::shutdown();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();

    return 0;
}
