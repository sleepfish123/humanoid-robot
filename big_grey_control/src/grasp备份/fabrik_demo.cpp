#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/JointState.h>
#include "big_grey_control/big_grey_srv.h"
#include<big_grey_control/RobotArm.h>
#include<big_grey_control/RobotClaw.h>
#include<big_grey_control/RobotWaist.h>
#include <big_grey_control/RobotCar.h>
#include <iostream>

using namespace std;

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

int flag = 0; // 接收标志位
int flag_server = 0;

// 弧度转角度
double rad2angle(double rad)
{
    double angle=rad*180.0/M_PI;
    return angle;
}

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
        cout << "************fabrik demo接收到的数据*******************" << endl;
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

bool fabrik_demo_func(big_grey_control::big_grey_srv::Request &req, big_grey_control::big_grey_srv::Response &res) {
    if (req.req.position[0] == 2.2) {
        // while (ros::ok() && flag == 0) {
        //     cout << "****************尚未接收到规划数据********************" << endl;
        //     usleep(1000);
        // }

        // 将目标构型发给moveit
        ROS_INFO("fabrik_server已响应");
        flag_server = 1;

        // 反馈响应
        res.res.name.clear();
        res.res.name.push_back("car_qianhou");
        res.res.name.push_back("car_zuoyou");
        res.res.name.push_back("rad_jiaobu");
        res.res.name.push_back("up_down_waist");
        res.res.name.push_back("rad_wanyao");
        res.res.name.push_back("rad_ceyao");
        res.res.name.push_back("rad_qianhoutai");
        res.res.name.push_back("rad_cetai");
        res.res.name.push_back("rad_dabi");
        res.res.name.push_back("rad_zhoubu");
        res.res.name.push_back("L1");
        res.res.name.push_back("L2");
        res.res.name.push_back("L3");

        res.res.position.clear();
        res.res.position.push_back(car_qianhou);
        res.res.position.push_back(car_zuoyou);
        res.res.position.push_back(rad_jiaobu);
        res.res.position.push_back(up_down_waist);
        res.res.position.push_back(rad_wanyao);
        res.res.position.push_back(rad_ceyao);
        res.res.position.push_back(rad_qianhoutai);
        res.res.position.push_back(rad_cetai);
        res.res.position.push_back(rad_dabi);
        res.res.position.push_back(rad_zhoubu);
        res.res.position.push_back(L1);
        res.res.position.push_back(L2);
        res.res.position.push_back(L3);

        for(int i=0;i<res.res.name.size();i++)
        {
            cout<<"***FABRIK CONTROL  "<<ends;
            cout<<res.res.name[i]<<"="
            <<res.res.position[i]<<"***"<<endl;
        }
    }
    else {
        ROS_INFO("fabrik control no receive request");
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fabrik_demo_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("send_rad", 1000, chatterCallback); // 监听send_rad话题，接收各关节运动量的数据
    ros::ServiceServer publish_pointcloud_service = n.advertiseService("fabrik_server", fabrik_demo_func);
    ROS_INFO("FABRIK Ready? ");

    // int i = 0;
    // while (i<1000) {
    //     usleep(10000);
    //     i += 1;
    //     ros::spinOnce();
    // }

    int fflag = 0;
    ros::Rate loop_rate(10);
    while (ros::ok() && fflag == 0) {
        if (flag_server==1 && flag==1) {
            cout << "****************moveit开始规划********************" << endl;
            fflag = 1;
            ros::AsyncSpinner spinner(1);
            spinner.start();

            moveit::planning_interface::MoveGroupInterface arm("zhuganzhi");

            arm.setGoalJointTolerance(0.001);

            arm.setMaxAccelerationScalingFactor(0.2);
            arm.setMaxVelocityScalingFactor(0.2);

            // // 控制机械臂先回到初始化位置
            // arm.setNamedTarget("init");
            // arm.move();
            // sleep(1);

            // double targetPose[17] = {0.0226247, 0.0128929, 0.0, 0.0, 
            // 0.517964, 0.00091007, 0.0, 0.0664117, // 0.402269
            // 1.46946, 0.244282, -0.179681, 1.57-0.976649,  // 1.57+0.765501（肘部）
            // 0.0, 0.0, 0.0, 0.0, 0.0};
            cout << "****************已接收到规划数据********************" << endl;
            std::vector<double> joint_group_positions(12);
            joint_group_positions[0] = car_qianhou; // car_y，y是前后
            joint_group_positions[1] = car_zuoyou; // car_x，x是左右
            joint_group_positions[2] = 0.0; // car_theta旋转
            joint_group_positions[3] = -rad_jiaobu; // foot注意注意注意注意脚部转动是顺时针为正
            joint_group_positions[4] = up_down_waist; // up_down_joint
            joint_group_positions[5] = -rad_wanyao; // bend_joint
            joint_group_positions[6] = -rad_ceyao; //  ceyao注意注意注意注意ceyao是顺时针为正
            joint_group_positions[7] = rad_qianhoutai; // 右臂前后
            joint_group_positions[8] = rad_cetai; // 右臂侧抬
            joint_group_positions[9] = rad_dabi; // 右大臂
            joint_group_positions[10] = rad_zhoubu + 3.1415926 / 2; // 右肘部+pi/2（发给moveit是弧度）
            joint_group_positions[11] = -30.0; // 右小臂

            arm.setJointValueTarget(joint_group_positions);
            arm.move();
            usleep(1000);
            cout << "****************结束规划********************" << endl;
            // ros::shutdown();
        }

        ros::spinOnce();
        loop_rate.sleep();

        if (fflag == 1) {
            cout << "****************grasp开始控制********************" << endl;
            // cout << "L1 = " << L1 << endl; // L1
            // cout << "L2 = " << L2 << endl; // L2
            // cout << "L3 = " << L3 << endl; // L3
            /*********弧度值转角度值***********/
            theta_zhoubu = rad2angle(rad_zhoubu);
            theta_dabi = rad2angle(rad_dabi);
            theta_cetai = rad2angle(rad_cetai);
            theta_qianhoutai = rad2angle(rad_qianhoutai);
            cout << "************GRASP控制前的数据*******************" << endl;
            cout << "car_qianhou = " << car_qianhou << endl; // car_qianhou
            cout << "car_zuoyou = " << car_zuoyou << endl; // car_zuoyou
            cout << "theta_jiaobu = " << rad2angle(rad_jiaobu) << endl; // rad_jiaobu
            cout << "up_down_waist = " << up_down_waist << endl; // up_down_waist
            cout << "theta_wanyao = " << rad2angle(rad_wanyao) << endl; // rad_wanyao
            cout << "theta_ceyao = " << rad2angle(rad_ceyao) << endl; // rad_ceyao
            cout << "theta_qianhoutai = " << theta_qianhoutai << endl; // rad_qianhoutai
            cout << "theta_cetai = " << theta_cetai << endl; // rad_cetai
            cout << "theta_dabi = " << theta_dabi << endl; // rad_dabi
            cout << "theta_zhoubu = " << theta_zhoubu << endl; // rad_zhoubu
            /***********************************/
            sensor_msgs::JointState robot_arm_states; // 控制手臂所需参数为sensor_msgs::JointState类型
            robot_arm_states.name.resize(13);
            robot_arm_states.name.push_back("angle_YXB");
            robot_arm_states.name.push_back("angle_YZB");
            robot_arm_states.name.push_back("angle_YDB");
            robot_arm_states.name.push_back("angle_YJQH");
            robot_arm_states.name.push_back("angle_YJCT");
            robot_arm_states.name.push_back("angle_ZXB");
            robot_arm_states.name.push_back("angle_ZZB");
            robot_arm_states.name.push_back("angle_ZDB");
            robot_arm_states.name.push_back("angle_ZJQH");
            robot_arm_states.name.push_back("angle_ZJCT");
            robot_arm_states.name.push_back("angle_ZSW");
            robot_arm_states.name.push_back("angle_DT");
            robot_arm_states.name.push_back("angle_YT");
            
            robot_arm_states.position.clear();
            robot_arm_states.position.push_back(30.0); // 右小臂（角度值）
            robot_arm_states.position.push_back(theta_zhoubu + 90); // 右肘部+90°
            robot_arm_states.position.push_back(theta_dabi); // 右大臂
            robot_arm_states.position.push_back(theta_cetai); // 右肩侧抬
            robot_arm_states.position.push_back(theta_qianhoutai); // 右肩前后
            robot_arm_states.position.push_back(0.0); // 点头
            robot_arm_states.position.push_back(0.0); // 摇头
            robot_arm_states.position.push_back(0.0); // 左肩前后
            robot_arm_states.position.push_back(0.0); // 左肩侧抬
            robot_arm_states.position.push_back(0.0); // 左大臂
            robot_arm_states.position.push_back(0.0); // 左肘部
            robot_arm_states.position.push_back(0.0); // 左小臂
            robot_arm_states.position.push_back(0.0); // 左手腕

            /******************************* 下发控制指令 *************************************/
            ROS_INFO("******************下发控制指令******************");
            // sleep(30);
            // 手臂
            RobotArm RobotArm_(ARM_SERIAL_DEFAULT_NAME, ARM_SERIAL_DEFAULT_BAUDRATE);
            // 爪子
            RobotClaw RobotClaw_(CLAW_SERIAL_DEFAULT_NAME,CLAW_SERIAL_DEFAULT_BAUDRATE);
            // 腰部
            int sockfd=socket(PF_INET, SOCK_STREAM,0);
            RobotWaist RobotWaist_(sockfd, WAIST_SENSOR_SERIAL_DEFAULT_NAME, WAIST_SENSOR_SERIAL_DEFAULT_BAUDRATE, FOOT_SENSOR_SERIAL_DEFAULT_NAME, FOOT_SENSOR_SERIAL_DEFAULT_BAUDRATE);
            theta_jiaobu = rad2angle(rad_jiaobu); // 脚部弧度转角度

            /*******************************去抓******************************/
            // 先手臂
            // RobotWaist_.foot_waist_action_control(theta_ceyao, theta_wanyao, (up_down_waist + 0.1) * 1000, theta_jiaobu); // 角度、mm
            RobotArm_.arm_control(robot_arm_states);
            
            // 再腰
            RobotWaist_.foot_waist_action_control(theta_ceyao, theta_wanyao, (up_down_waist + 0.03) * 1000, (theta_jiaobu + 6)); // 角度、mm
            // 脚部逆时针旋转补偿12°
            
            // sleep(10); // 确保手臂运动到了再闭合爪子
            // 再爪子
            RobotClaw_.claw_close_control(); // 爪子闭合
            usleep(1000);

            // sleep(60); // ************************
            /*******************************抓完回来******************************/
            // 先手臂
            RobotArm_.arm_init(); // 手臂初始化
            // 再腰
            // RobotWaist_.foot_waist_action_control(-theta_ceyao, -theta_wanyao, -(up_down_waist + 0.1) * 1000, -theta_jiaobu); // 角度、mm
            RobotWaist_.foot_waist_action_control(-theta_ceyao, -theta_wanyao, -(up_down_waist + 0.03) * 1000, 0.0); // 角度、mm
            
            // 最后爪子张开
            RobotClaw_.claw_open_control(); // 爪子张开
            ROS_INFO("完成完成完成完成完成完成完成完成完成完成完成完成");
            sleep(10); // ********************************
            ros::shutdown(); 
        }
    }

    // 控制机械臂先回到初始化位置
    // arm.setNamedTarget("init");
    // arm.move();
    // sleep(1);

    // ros::shutdown(); 
    ros::spin();

    return 0;
}
