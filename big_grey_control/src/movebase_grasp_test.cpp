#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "actionlib_msgs/GoalStatus.h"

/**
  * @file movebase_grasp_test.cpp
  * @brief  接收到导航结束的指令后开始执行抓取程序，抓取结束后反馈给导航抓取成功
  * @version  1.0
  * @data 2023-01-05
  */

std_msgs::Int16 grasp_start; // 开始执行抓取的标志位
std_msgs::Int16 grasp_finish; // 抓取执行成功后返回给导航的值
// actionlib_msgs::GoalStatus received_data;

void chatterCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
  // ROS_INFO("I heard status : [%d]", msg->status_list[11]);
  int size = sizeof(msg->status_list);
  uint8_t status = msg->status_list[size - 1].status;
  if (status == 3) { // 导航结束，已到达目标点
    grasp_start.data = 1;
    ROS_INFO("status = 3, has reached the goal");
  }
  else { // 
    ROS_INFO("status = %d, not reach the goal", status);
  }

  return;
}

int main(int argc, char **argv)
{
    // 初始化
    grasp_start.data = 0;
    grasp_finish.data = 0;

    ros::init(argc, argv, "movebase_grasp_test_node"); // 节点名
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/move_base/status", 1000, chatterCallback);
    ros::Publisher pub = n.advertise<std_msgs::Int16>("movebase_grasp_test", 1000);

    // ros::Rate loop_rate(10);
    while (ros::ok())
    {
        // ROS_INFO("data : %d", data);

        // 业务处理
        if (grasp_start.data == 1) {

          ROS_INFO("start grasp...");
          sleep(5); // 业务处理
          ROS_INFO("finish grasp...");

          grasp_finish.data = 1;
          pub.publish(grasp_finish);
          pub.publish(grasp_finish);
          pub.publish(grasp_finish);
          pub.publish(grasp_finish);
          pub.publish(grasp_finish);

          grasp_start.data = 0;
        }
        else {
          grasp_finish.data = 0;
          pub.publish(grasp_finish);
          ROS_INFO("WAITING FOR GRASP...");
        }

        ros::spinOnce();

        // loop_rate.sleep();
    }

    return 0;
}