#include <serial/serial.h>  
#include <sensor_msgs/JointState.h>
#include<big_grey_control/RobotCarControl.h>
#include <ros/ros.h>
#include <iostream>
#include <unistd.h> // 延时函数头文件 usleep(int a); // 延迟a微秒
#include "big_grey_control/big_grey_srv.h"

using namespace std;

int socket_fd = socket(AF_INET, SOCK_STREAM, 0);

bool car_init(big_grey_control::big_grey_srv::Request &req, big_grey_control::big_grey_srv::Response &res) {
    if (req.req.position[0] == 2.2) {
        /*******************************小车初始化***********************************/

        int socketfd = socket(PF_INET, SOCK_STREAM, 0);
        // 小车多点运行，发一次
        RobotCarControl rcc(socketfd);
        rcc.MultiplePoints(1, 0, -1.2, 0.0, 
                                        0.0, 0.0, 0.0, 
                                        5.0, 1);

        // //创建客户端套接字
	    // if (socket_fd == -1) {
        //     ROS_INFO("socket 创建失败");
        //     exit(-1);
        // }
        // RobotCar RobotCar_(socket_fd);
        // // 初始化时小车前后左右运动量
        // double car_forward_length = 5.0;
        // double car_back_length = 5.0;
        // double car_left_length = 5.0;
        // double car_right_length = 5.0;
        // RobotCar_.car_forward_control(car_forward_length);
        // sleep(2);
        // RobotCar_.car_back_control(car_back_length);
        // sleep(2);
        // RobotCar_.car_left_control(car_left_length);
        // sleep(2);
        // RobotCar_.car_right_control(car_right_length);
        sleep(2);
        /***********************************************************************************/

        // 反馈响应
        res.res.name.clear();
        res.res.name.push_back("car_init");

        res.res.position.clear();
        res.res.position.push_back(4.4);

        for(int i=0;i<res.res.name.size();i++)
        {
            cout<<"***CAR INIT  "<<ends;
            cout<<res.res.name[i]<<"="
            <<res.res.position[i]<<"***"<<endl;
        }
    }
    else {
        ROS_INFO("car init no receive request");
    }

    return true;
}

int main(int argc, char **argv) {
    setlocale(LC_ALL,""); // 中文输出
    ros::init(argc, argv, "car_init_node");
    ros::NodeHandle n;
    ros::ServiceServer car_init_service = n.advertiseService("car_init_server", car_init);
    ROS_INFO("Car Init Ready? ");
    ros::spin();

    return 0;
}
