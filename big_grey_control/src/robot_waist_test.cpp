#include <ros/ros.h>
#include<big_grey_control/RobotWaist.h>
#include<big_grey_control/Robot.h>

void waist_foot_test()
{
  // 腰部
  int sockfd=socket(PF_INET, SOCK_STREAM,0);
  // RobotWaist RobotWaist_(sockfd,WAIST_SENSOR_SERIAL_DEFAULT_NAME,WAIST_SENSOR_SERIAL_DEFAULT_BAUDRATE);
  RobotWaist RobotWaist_(sockfd, WAIST_SENSOR_SERIAL_DEFAULT_NAME, WAIST_SENSOR_SERIAL_DEFAULT_BAUDRATE,
                          FOOT_SENSOR_SERIAL_DEFAULT_NAME, FOOT_SENSOR_SERIAL_DEFAULT_BAUDRATE);
  // RobotWaist_.foot_sensor_clear(); // 足部传感器清零

  double theta_jiaobu = 90.0; // -4.99
  double theta_ceyao = 0.0;
  double theta_wanyao = 0.0;
  double up_down_waist = 0.1; // m

  // 腰和足
  RobotWaist_.foot_waist_action_control(theta_ceyao, theta_wanyao, up_down_waist * 1000, theta_jiaobu); // 角度、mm
  // int flag;
  // cin >> flag;
  // RobotWaist_.foot_waist_action_control(-theta_ceyao, -theta_wanyao, -up_down_waist * 1000, -theta_jiaobu); // 角度、mm
}

int main(int argc, char *argv[])
{
  waist_foot_test();
  return 0;
}