#include "big_grey_control/servo_command.h"
#include "big_grey_control/big_grey_srv.h"
#include <big_grey_control/RobotArm.h>
#include <big_grey_control/RobotClaw.h>
#include <unistd.h> // 延时函数头文件 usleep(int a); // 延迟a微秒
#include <serial/serial.h>  
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <iostream>

using namespace std;

bool manipulator_init(big_grey_control::big_grey_srv::Request &req, big_grey_control::big_grey_srv::Response &res) {
    if (req.req.position[0] == 2.2) {
        RobotArm RobotArm_(ARM_SERIAL_DEFAULT_NAME, ARM_SERIAL_DEFAULT_BAUDRATE);
        RobotClaw RobotClaw_(CLAW_SERIAL_DEFAULT_NAME,CLAW_SERIAL_DEFAULT_BAUDRATE);
        
        /*******************************机械臂初始化***********************************/
        RobotArm_.arm_init();
        /***********************************************************************************/

        /*******************************夹爪初始化***********************************/
        RobotClaw_.claw_open_control();
        sleep(1);
        RobotClaw_.claw_close_control();
        sleep(1);
        /***********************************************************************************/

        // 反馈响应
        res.res.name.clear();
        res.res.name.push_back("manipulator_init");

        res.res.position.clear();
        res.res.position.push_back(4.4);

        for(int i=0;i<res.res.name.size();i++)
        {
            cout<<"***MANIPULATOR INIT  "<<ends;
            cout<<res.res.name[i]<<"="
            <<res.res.position[i]<<"***"<<endl;
        }
    }
    else {
        ROS_INFO("manipulator init no receive request");
    }

    return true;
}

int main(int argc, char** argv) {
    setlocale(LC_ALL,""); // 中文输出
    ros::init(argc,argv,"manipulator_init_node");
    ros::NodeHandle n;
    
    // 定义服务器
    ros::ServiceServer robot_service = n.advertiseService("manipulator_init_server", manipulator_init);
    ROS_INFO("Manipulator Init Ready? ");
    // ros::Rate loop_rate(1);
    // 处理ROS的信息，比如订阅消息，并调用回调函数 
    ros::spin();
    // loop_rate.sleep(); 

    return 0;
}