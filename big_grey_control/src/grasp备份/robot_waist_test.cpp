#include <ros/ros.h>
#include<big_grey_control/RobotWaist.h>
void waist_test()
{
  double Gan[3]={0};
  double target_Gan[3]={6,4,6};
  int sockfd=socket(PF_INET, SOCK_STREAM,0);
  RobotWaist R(sockfd,WAIST_SENSOR_SERIAL_DEFAULT_NAME,WAIST_SENSOR_SERIAL_DEFAULT_BAUDRATE);
  while (1)
  {
    R.read_waist_sensor(Gan);
    for(int i=0;i<3;i++)
    {
      ROS_INFO_STREAM("Link "<<i<<" : "<<Gan[i]);
    }
  }
  R.waist_action_control(-target_Gan[0],-target_Gan[1],-target_Gan[2]);
  R.read_waist_sensor(Gan);
  for(int i=0;i<3;i++)
  {
    ROS_INFO_STREAM("Link "<<i<<" : "<<Gan[i]<<endl);
  }
  // sleep(2);
  // R.waist_action_control(-target_Gan[0],-target_Gan[1],-target_Gan[2]);
}
void waist_test2()
{
  int sockfd=socket(PF_INET, SOCK_STREAM,0);
  RobotWaist R(sockfd);
  R.foot_waist_action_control(0,0,-100,0);
  R.foot_waist_action_control(0,20,0,0);
  sleep(8);
  R.foot_waist_action_control(0,-20,0,0);

}
void foot_test()
{
  // R.foot_sensor_clear();
  // usleep(100*1000);
  int sockfd=socket(PF_INET, SOCK_STREAM,0);
  RobotWaist R(sockfd,FOOT_SENSOR_SERIAL_DEFAULT_BAUDRATE,FOOT_SENSOR_SERIAL_DEFAULT_NAME);
  while (1)
  {
    ROS_INFO_STREAM("足部角度是"<<R.read_foot_sensor());
  }
  R.foot_action_control(-5.0);
  // sleep(1);
  // R.foot_action_control(0.0);
}

void waist_foot_test()
{
  double Gan[3]={0};
  double target_Gan[3]={6,4,6};
  int sockfd=socket(PF_INET, SOCK_STREAM,0);
  RobotWaist R(sockfd,WAIST_SENSOR_SERIAL_DEFAULT_NAME,WAIST_SENSOR_SERIAL_DEFAULT_BAUDRATE,
  FOOT_SENSOR_SERIAL_DEFAULT_NAME,FOOT_SENSOR_SERIAL_DEFAULT_BAUDRATE);
  // R.foot_action_control(-20.0);
  // R.waist_action_control(target_Gan[0],target_Gan[1],target_Gan[2]);
  R.read_waist_sensor(Gan);
  for(int i=0;i<3;i++)
  {
    ROS_INFO_STREAM("Link "<<i<<" : "<<Gan[i]<<endl);
  }
  int flag;
  cout<<"waist flag= "<<endl;
  cin>>flag;
  R.foot_action_control(0.0);
  R.waist_action_control(-target_Gan[0],-target_Gan[1],-target_Gan[2]);
}

int main(int argc, char *argv[])
{
  foot_test();
  return 0;
}