#include <iostream>
#include "ros/ros.h"
#include "big_grey_control/big_grey_srv.h"
#include <big_grey_control/RobotArm.h>
#include <big_grey_control/RobotClaw.h>
#include "sensor_msgs/JointState.h"
#include <cstdlib>
#include "std_msgs/Float32MultiArray.h"
#include <unistd.h> // 延时函数头文件 usleep(5); // 延迟5us

// C++启动shell脚本所需头文件
#include <stdio.h>   //popen()
#include <string.h>  //memset()

using namespace std;

int main(int argc, char **argv)
{
  setlocale(LC_ALL,""); // 中文输出
  ros::init(argc, argv, "big_grey_brain_node");
  ros::NodeHandle n;
  // 定义客户端，请求相应服务端
  ros::ServiceClient manipulator_init_client = n.serviceClient<big_grey_control::big_grey_srv>("manipulator_init_server"); // 机械臂初始化
  // ros::ServiceClient car_init_client = n.serviceClient<big_grey_control::big_grey_srv>("car_init_server");
  // ros::ServiceClient waist_init_client = n.serviceClient<big_grey_control::big_grey_srv>("waist_init_server");
  ros::ServiceClient object_detection_client = n.serviceClient<big_grey_control::big_grey_srv>("object_detection_server"); // 目标识别
  ros::ServiceClient publish_pointcloud_client = n.serviceClient<big_grey_control::big_grey_srv>("publish_pointcloud_server"); // 发送点云数据
  ros::ServiceClient fabrik_client = n.serviceClient<big_grey_control::big_grey_srv>("fabrik_server"); // FABRIK求逆解
  ros::ServiceClient grasp_client = n.serviceClient<big_grey_control::big_grey_srv>("grasp_server"); // 控制各关节运动

  /*************************************************物体识别作为服务端******************************************/
  system("gnome-terminal -e /home/yu/big_grey/src/shell/object_detection.sh"); // 开物体识别
  
  return 0;
}
