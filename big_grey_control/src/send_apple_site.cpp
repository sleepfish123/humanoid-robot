/****** 串口控制舵机所需头文件 ******/
#include <serial/serial.h>  
#include <ros/ros.h>
#include <iostream>
#include <unistd.h> // 延时函数头文件 usleep(5); // 延迟5us
// #include <std_msgs/Float32.h>
#include "std_msgs/Float32MultiArray.h"
#include <sensor_msgs/JointState.h>
#include "big_grey_control/big_grey_srv.h"

using namespace std;

serial::Serial ser; // 声明串口对象
#define PI 3.1415926

// double apple_site_x = 1.1, apple_site_y = 2.2, apple_site_z = 3.3;
double apple_site_x, apple_site_y, apple_site_z;
int flag_receive_apple_site = 0;

void apple_site_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {    
    // cout << "待定x = " << msg->data.at(0) << endl;
    // cout << "待定y = " << msg->data.at(1) << endl;
    // cout << "待定z = " << msg->data.at(2) << endl;
    ROS_INFO("进入回调函数，接收apple_site");
    apple_site_x = msg->data.at(0);
    apple_site_y = msg->data.at(1);
    apple_site_z = msg->data.at(2);
    flag_receive_apple_site = 1;
}

bool read_apple_site(big_grey_control::big_grey_srv::Request &req, big_grey_control::big_grey_srv::Response &res) {
    if (req.req.position[0] == 2.2) {

        while (flag_receive_apple_site == 0) {
            ROS_INFO("未接收到apple_site");
            ros::spinOnce();
            usleep(10000);
        }

        // 赋值
        res.res.name.clear();
        res.res.name.push_back("apple_site_x");
        res.res.name.push_back("apple_site_y");
        res.res.name.push_back("apple_site_z");

        res.res.position.clear();
        res.res.position.push_back(apple_site_x);
        res.res.position.push_back(apple_site_y);
        res.res.position.push_back(apple_site_z);

        // 物体识别整个过程大概1s时间
        sleep(1);
        
        // 打印输出
        for(int i=0;i<res.res.name.size();i++)
        {
            cout<<"***DEBUGOUTPUT  "<<ends;
            cout<<res.res.name[i]<<"="
            <<res.res.position[i]<<"***"<<endl;
        }
    }
    else {
        ROS_INFO("object detection no receive request");
    }

    return true;
}

int main(int argc, char** argv) {
    setlocale(LC_ALL,""); // 中文输出
    ros::init(argc, argv, "object_detection_node");
    ros::NodeHandle n;
    // 订阅器
    ros::Subscriber apple_site_sub = n.subscribe("apple_site", 1000, apple_site_callback);
    // 服务端
    ros::ServiceServer robot_service = n.advertiseService("object_detection_server", read_apple_site);
    ROS_INFO("Object Detection Ready? ");

    ros::spin();

    return 0;
}
