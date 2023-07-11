#include <iostream>
#include "ros/ros.h"
#include "big_grey_control/big_grey_srv.h"
#include <big_grey_control/RobotArm.h>
#include <big_grey_control/RobotClaw.h>
#include <big_grey_control/RobotWaist.h>
#include <big_grey_control/RobotCarControl.h>
#include "sensor_msgs/JointState.h"
#include <cstdlib>
#include "std_msgs/Float32MultiArray.h"
#include <unistd.h> // 延时函数头文件 usleep(5); // 延迟5us

/********** 小车导航头文件 ***********/
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
/*************************************/

// C++启动shell脚本所需头文件
#include <stdio.h>   //popen()
#include <string.h>  //memset()

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  MoveBaseClient;

int main(int argc, char **argv)
{
  setlocale(LC_ALL,""); // 中文输出
  ros::init(argc, argv, "big_grey_brain_node");
  ros::NodeHandle n;
  // 定义客户端，请求相应服务端
  ros::ServiceClient manipulator_init_client = n.serviceClient<big_grey_control::big_grey_srv>("manipulator_init_server"); // 机械臂初始化
  ros::ServiceClient car_init_client = n.serviceClient<big_grey_control::big_grey_srv>("car_init_server");
  // ros::ServiceClient waist_init_client = n.serviceClient<big_grey_control::big_grey_srv>("waist_init_server");
  ros::ServiceClient object_detection_client = n.serviceClient<big_grey_control::big_grey_srv>("object_detection_server"); // 目标识别
  ros::ServiceClient publish_pointcloud_client = n.serviceClient<big_grey_control::big_grey_srv>("publish_pointcloud_server"); // 发送点云数据
  ros::ServiceClient fabrik_client = n.serviceClient<big_grey_control::big_grey_srv>("fabrik_server"); // FABRIK求逆解
  ros::ServiceClient grasp_client = n.serviceClient<big_grey_control::big_grey_srv>("grasp_server"); // 控制各关节运动

  /****************************** 小车导航 *****************************/
  MoveBaseClient ac("move_base",true);
  ROS_INFO("Waiting for the move_base action server");
  ac.waitForServer(ros::Duration(1));
  // Send a goal to move_base 发送目标到move_base节点
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 3.0;
  goal.target_pose.pose.position.y = 0.5;
  goal.target_pose.pose.orientation.w = 1;
  ROS_INFO("Sending goal");   
  ac.sendGoal(goal);
  // Wait for the action to return
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("You have reached the goal!");
  else
    ROS_INFO("The base failed for some reason");

  // return 0;
  
  /*****************************************************************************************/

  // /*************************************************机械臂初始化******************************************/
  RobotArm RobotArm_(ARM_SERIAL_DEFAULT_NAME, ARM_SERIAL_DEFAULT_BAUDRATE);// 2222222222222222222
  RobotArm_.arm_init();// 2222222222222222222

  // RobotClaw RobotClaw_(CLAW_SERIAL_DEFAULT_NAME,CLAW_SERIAL_DEFAULT_BAUDRATE); // 2222222222222222222
  // // RobotClaw_.claw_close_control();
  // // sleep(3);
  // RobotClaw_.claw_open_control();
  // sleep(1);

  int sockfd=socket(PF_INET, SOCK_STREAM,0);
  RobotWaist RobotWaist_(sockfd, WAIST_SENSOR_SERIAL_DEFAULT_NAME, WAIST_SENSOR_SERIAL_DEFAULT_BAUDRATE,
                          FOOT_SENSOR_SERIAL_DEFAULT_NAME, FOOT_SENSOR_SERIAL_DEFAULT_BAUDRATE);
  RobotWaist_.foot_sensor_clear(); // 足部传感器清零

  /************************ 小车初始化 ****************************/
  int sockfd1=socket(PF_INET, SOCK_STREAM,0);
  // 小车多点运行，发一次
  RobotCarControl rcc(sockfd1);
  rcc.MultiplePoints(1, -1.2, 0.0, 0.0, 
                                  0.0, 0.0, 0.0, 
                                  6.0, 1);

  

  // usleep(1000);

  // RobotClaw RobotClaw_(CLAW_SERIAL_DEFAULT_NAME,CLAW_SERIAL_DEFAULT_BAUDRATE);
  // RobotClaw_.claw_open_control();
  // sleep(10);
  // big_grey_control::big_grey_srv control_manipulator_init;
  // control_manipulator_init.request.req.position.clear();
  // control_manipulator_init.request.req.position.push_back(2.2);

  // ROS_INFO("机械臂初始化请求已发送");
  // ros::service::waitForService("/manipulator_init_server");
  // manipulator_init_client.call(control_manipulator_init);

  // // 打印输出
  // for(int manipulator_init_i=0;manipulator_init_i<control_manipulator_init.response.res.name.size();manipulator_init_i++)
  // {
  //   cout<<"*** MANIPULATOR INIT  "<<ends;
  //   cout<<control_manipulator_init.response.res.name[manipulator_init_i]<<"="
  //   <<control_manipulator_init.response.res.position[manipulator_init_i]<<"***"<<endl;
  // }

  // // /*********************************************************************************************************/

  /*************************************************小车初始化**********************************************/
  // big_grey_control::big_grey_srv control_car_init;
  // control_car_init.request.req.position.clear();
  // control_car_init.request.req.position.push_back(2.2);

  // ROS_INFO("小车初始化请求已发送");
  // ros::service::waitForService("/car_init_server");
  // car_init_client.call(control_car_init);

  // // 打印输出
  // for(int car_init_i=0;car_init_i<control_car_init.response.res.name.size();car_init_i++)
  // {
  //   cout<<"*** CAR INIT  "<<ends;
  //   cout<<control_car_init.response.res.name[car_init_i]<<"="
  //   <<control_car_init.response.res.position[car_init_i]<<"***"<<endl;
  // }
  /*********************************************************************************************************/

  // /*************************************************腰部初始化**********************************************/
  // big_grey_control::big_grey_srv control_waist_init;
  // control_waist_init.request.req.position.clear();
  // control_waist_init.request.req.position.push_back(2.2);

  // ROS_INFO("腰部初始化请求已发送");
  // ros::service::waitForService("/waist_init_server");
  // waist_init_client.call(control_waist_init);

  // // 打印输出
  // for(int waist_init_i=0;waist_init_i<control_waist_init.response.res.name.size();waist_init_i++)
  // {
  //   cout<<"*** WAIST INIT  "<<ends;
  //   cout<<control_waist_init.response.res.name[waist_init_i]<<"="
  //   <<control_waist_init.response.res.position[waist_init_i]<<"***"<<endl;
  // }
  /*********************************************************************************************************/

  /*************************************************物体识别作为服务端******************************************/
  // system("gnome-terminal -e /home/yu/big_grey/src/shell/test_send_apple_site.sh"); // 开物体识别

  // 打开新终端，开物体识别
  system("gnome-terminal -e /home/yu/big_grey/src/shell/object_detection.sh");
  // sleep(5);
  
  // // system("gnome-terminal -e /home/yu/big_grey/src/shell/send_apple_site.sh"); // 测试用

  // big_grey_control::big_grey_srv control_object_detection;
  // control_object_detection.request.req.position.clear();
  // control_object_detection.request.req.position.push_back(2.2);

  // ROS_INFO("物体识别请求已发送");
  // ros::service::waitForService("/object_detection_server");
  // object_detection_client.call(control_object_detection);

  // // 打印输出
  // for(int od_i=0;od_i<control_object_detection.response.res.name.size();od_i++)
  // {
  //   cout<<"*** OBJECT DETECTION  "<<ends;
  //   cout<<control_object_detection.response.res.name[od_i]<<"="
  //   <<control_object_detection.response.res.position[od_i]<<"***"<<endl;
  // }

  // // 接收apple_site_x,apple_site_y,apple_site_z
  // double apple_site_x = control_object_detection.response.res.position[0]; // 单位（）
  // double apple_site_y = control_object_detection.response.res.position[1];
  // double apple_site_z = control_object_detection.response.res.position[2];

  /************************************************************************************************************/

  /*************************************************新算法作为服务端******************************************/
  // 打开服务端
  system("gnome-terminal -e /home/yu/big_grey/src/shell/publish_pointcloud.sh");

  big_grey_control::big_grey_srv control_publish_pointcloud;
  control_publish_pointcloud.request.req.position.clear();
  control_publish_pointcloud.request.req.position.push_back(2.2);

  ROS_INFO("发布点云请求已发送");
  ros::service::waitForService("/publish_pointcloud_server");
  publish_pointcloud_client.call(control_publish_pointcloud);

  // 打印输出
  // ppc = publish_pointcloud
  for(int ppc_i=0;ppc_i<control_publish_pointcloud.response.res.name.size();ppc_i++)
  {
    cout<<"*** PUBLISH POINTCLOUD  "<<ends;
    cout<<control_publish_pointcloud.response.res.name[ppc_i]<<"="
    <<control_publish_pointcloud.response.res.position[ppc_i]<<"***"<<endl;
  }
  /************************************************************************************************************/

  // /*************************************************启动moveit、点云转八叉树、FABRIK求逆解******************************************/
  // // FILE *fp;
	// // char buffer[80];
	// // memset(buffer, 0x00, sizeof(buffer));
 
	// // fp = popen("/home/yu/big_grey/src/big_grey_control/src/big_grey_part.sh", "r");
	// // fgets(buffer, sizeof(buffer), fp);
 
	// // printf("[%s]\n", buffer);
 
	// // pclose(fp);

  // // system("gnome-terminal -e ./test");

  // system("gnome-terminal -e /home/yu/big_grey/src/shell/test_send_apple_site.sh"); // 开物体识别
  system("gnome-terminal -e /home/yu/big_grey/src/shell/big_grey_part1.sh");
  system("gnome-terminal -e /home/yu/big_grey/src/shell/big_grey_part2.sh");

  // system("gnome-terminal -e /home/yu/big_grey/src/shell/object_detection.sh"); // 开物体识别
  // system("gnome-terminal -e /home/yu/big_grey/src/shell/test_send_apple_site.sh"); // 开物体识别***************************
  
  // sleep(10);
  // /************************************************************************************************************/

  /*************************************************fabrik给moveit目标构型作为服务端******************************************/
  system("gnome-terminal -e /home/yu/big_grey/src/shell/fabrik_demo.sh");

  big_grey_control::big_grey_srv control_fabrik;
  control_fabrik.request.req.position.clear();
  control_fabrik.request.req.position.push_back(2.2);

  ROS_INFO("fabrik给moveit目标构型请求已发送");
  ros::service::waitForService("/fabrik_server");
  fabrik_client.call(control_fabrik);

  // 打印输出，将参数传给数组
  // ppc = publish_pointcloud
  for(int fabrik_i=0;fabrik_i<control_fabrik.response.res.name.size();fabrik_i++)
  {
    cout<<"*** FABRIK CONGIURATION TO MOVEIT  "<<ends;
    cout<<control_fabrik.response.res.name[fabrik_i]<<"="
    <<control_fabrik.response.res.position[fabrik_i]<<"***"<<endl;
  }
  /************************************************************************************************************/

  // /*******************************************抓取控制grasp***************************************************/
  // // sleep(30); // 等待逆运动学求解完毕
  // system("gnome-terminal -e /home/yu/big_grey/src/shell/grasp.sh");

  // big_grey_control::big_grey_srv control_grasp;
  // control_grasp.request.req.name.clear();
  // control_grasp.request.req.name.push_back("flag"); // 加上flag共11个参数
  // control_grasp.request.req.name.push_back("car_qianhou");
  // control_grasp.request.req.name.push_back("car_zuoyou");
  // control_grasp.request.req.name.push_back("rad_jiaobu");
  // control_grasp.request.req.name.push_back("up_down_waist");
  // control_grasp.request.req.name.push_back("rad_wanyao");
  // control_grasp.request.req.name.push_back("rad_ceyao");
  // control_grasp.request.req.name.push_back("rad_qianhoutai");
  // control_grasp.request.req.name.push_back("rad_cetai");
  // control_grasp.request.req.name.push_back("rad_dabi");
  // control_grasp.request.req.name.push_back("rad_zhoubu");
  // control_grasp.request.req.name.push_back("L1");
  // control_grasp.request.req.name.push_back("L2");
  // control_grasp.request.req.name.push_back("L3");

  // control_grasp.request.req.position.clear();
  // control_grasp.request.req.position.push_back(2.2);
  // for(int grasp_i1=0;grasp_i1<control_fabrik.response.res.name.size();grasp_i1++)
  // {
  //   control_grasp.request.req.position.push_back(control_fabrik.response.res.position[grasp_i1]);
  //   cout<<"*** GRASP DATA  "<<ends;
  //   cout<<control_grasp.request.req.name[grasp_i1]<<"="
  //   <<control_grasp.request.req.position[grasp_i1]<<"***"<<endl;
  // }

  // ROS_INFO("抓取请求已发送");
  // ros::service::waitForService("/grasp_server");
  // grasp_client.call(control_grasp);

  // // 打印输出
  // for(int grasp_i2=0;grasp_i2<control_grasp.response.res.name.size();grasp_i2++)
  // {
  //   cout<<"*** GRASP  "<<ends;
  //   cout<<control_grasp.response.res.name[grasp_i2]<<"="
  //   <<control_grasp.response.res.position[grasp_i2]<<"***"<<endl;
  // }
  // /************************************************************************************************************/
  

  ROS_INFO("*************mission complete*************");
  // sleep(60);

  // // /*************************************************机械臂作为服务端******************************************/
  // // ros::ServiceClient manipulator_client = n.serviceClient<small_ju_brain::small_ju_brain>("manipulator_server"); // 定义客户端，请求相应服务端
  // // small_ju_brain::small_ju_brain control_manipulator;
  // // control_manipulator.request.req = 1;

  // // ros::service::waitForService("/manipulator_server");
  // // ROS_INFO("机械臂请求已发送");
  // // manipulator_client.call(control_manipulator);

  // // // 打印输出
  // // for(int od_i=0;od_i<control_object_detection.response.res.name.size();od_i++)
  // // {
  // //   cout<<"*** OBJECT DETECTION  "<<ends;
  // //   cout<<control_object_detection.response.res.name[od_i]<<"="
  // //   <<control_object_detection.response.res.position[od_i]<<"***"<<endl;
  // // }
  // // /************************************************************************************************************/

  // // if (client.call(srv))
  // // {
  // //   ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  // // }
  // // else
  // // {
  // //   ROS_ERROR("Failed to call service add_two_ints");
  // //   return 1;
  // // }

  // // 小车回原位
  // rcc.MultiplePoints(1, 0, 1.2, 0.0, 
  //                                 0.0, 0.0, 0.0, 
  //                                 5.0, 1);

  return 0;
}
