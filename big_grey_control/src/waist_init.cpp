#include <serial/serial.h>  
#include <sensor_msgs/JointState.h>
#include<big_grey_control/RobotWaist.h>
#include <ros/ros.h>
#include <iostream>
#include <unistd.h> // 延时函数头文件 usleep(int a); // 延迟a微秒
#include "big_grey_control/big_grey_srv.h"

using namespace std;

double angle2rad(double angle)
{
    double rad=angle*M_PI/180.0;
    return rad;
}

bool waist_init(big_grey_control::big_grey_srv::Request &req, big_grey_control::big_grey_srv::Response &res) {
    if (req.req.position[0] == 2.2) {
        /*******************************腰部初始化***********************************/
        //创建客户端套接字
	    int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
        RobotWaist RobotWaist_(socket_fd);
        // 初始化时腰部各关节运动量
        double foot_theta = 30.0; // 脚部旋转角度
        // 定义L1、L2、L3
        double L1, L2, L3; // 转换对应高度
        L1 = 5;
        L2 = 5;
        L3 = 5;

        
        // // 弯腰侧腰测试
        // // 定义初始套筒高度L0，圆盘半径R，以及L1、L2、L3
        // double L0 = 0.64244;
        // double R = 0.09805; // 单位：m
        // double theta6 = 30.0; // qian+???
        // double theta7 = 30.0; // zuo+???
        // double cos_6 = cos(angle2rad(theta6));
        // double sin_6 = sin(angle2rad(theta6));
        // double cos_7 = cos(angle2rad(theta7));
        // double sin_7 = sin(angle2rad(theta7));
        // double up_down_waist = 0.0; // 不上升也不下降，但是要记住先上升一定距离以后再弯腰/侧腰
        // L1 = -L0+sqrt(pow(-R+R*cos_6,2)+pow(R*sin_6+L0+up_down_waist,2));
        // L2 = -L0+sqrt(pow(0.5*R-0.5*R*cos_6+root_3*0.5*R*sin_6*sin_7,2)+pow (-1*root_3*0.5*R+root_3*0.5*R*cos_7,2)+pow(-0.5*R*sin_6-root_3*0.5*R*sin_7*cos_6+L0+up_down_waist,2));
        // L3 = -L0+sqrt(pow(0.5*R-0.5*R*cos_6-root_3*0.5*R*sin_6*sin_7,2)+pow (root_3*0.5*R-root_3*0.5*R*cos_7,2)+pow(-0.5*R*sin_6+root_3*0.5*R*sin_7*cos_6+L0+up_down_waist,2));


        // RobotWaist_.foot_action_control(foot_theta);
        // sleep(20);
        RobotWaist_.waist_action_control(L1, L2, L3); // 输入cm
        sleep(20);
        // RobotWaist_.foot_action_control(-foot_theta);
        // sleep(1);
        // RobotWaist_.waist_action_control(-l1, -l2, -l3);
        /***********************************************************************************/

        // 反馈响应
        res.res.name.clear();
        res.res.name.push_back("waist_init");

        res.res.position.clear();
        res.res.position.push_back(4.4);

        for(int i=0;i<res.res.name.size();i++)
        {
            cout<<"***WAIST INIT  "<<ends;
            cout<<res.res.name[i]<<"="
            <<res.res.position[i]<<"***"<<endl;
        }
    }
    else {
        ROS_INFO("waist init no receive request");
    }

    return true;
}

int main(int argc, char **argv) {
    setlocale(LC_ALL,""); // 中文输出
    ros::init(argc, argv, "waist_init_node");
    ros::NodeHandle n;
    ros::ServiceServer car_init_service = n.advertiseService("waist_init_server", waist_init);
    ROS_INFO("Waist Init Ready? ");
    ros::spin();

    return 0;
}
