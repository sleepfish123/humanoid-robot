/*
* 文件名：car_test.cpp
* 描述：大灰小车测试代码，测试小车多点运行命令
* 修改人：于川
* 修改时间：2022-09-08
* 修改内容：新增
*/

#include <big_grey_control/RobotCarControl.h>

using namespace std;

void car_test()
{

#if 0

  // 测试成功：小车轮子转动
  RobotCarControl rcc(socketfd);
  while (1) {
    rcc.SetSpeed(0);
  }
  
#elif 0

  // 小车多点运行，一直发
  RobotCarControl rcc(socketfd);

  while (1) {
    rcc.MultiplePoints(1, 0.0, 0.957, 0.0, 
                                  0.0, 0.1914, 0.0, 
                                  5.0, 1);
  }

#else

#if WIFI_CONTROL

  int socketfd = socket(PF_INET, SOCK_STREAM, 0);
  RobotCarControl rcc(socketfd);

#elif SERIAL_CONTROL

  RobotCarControl rcc;

#elif

  cout << "neither WIFI_CONTROL nor SERIAL_CONTROL" << endl;

#endif

  // 小车多点运行，发一次
  int flag = 0;
  int count = 1;
  for (;;) {
    rcc.MultiplePoints(1, 
                                  1.2, 0.0, 0.0, 
                                  0.0, 0.0, 0.0, 
                                  6.0, count);
    count++;
    cout << "小车是否继续运动";
    cin >> flag;
  }
  
  

  // void RobotCarControl::MultiplePoints(int pointNum, double positionX, double positionY, double positionZ, 
  //                                                                               double velocityX, double velocityY, double velocityZ, double timeInterval, int num) {

#endif
  
  // sleep(5);
  
  // int pointNum = 1;
  // double positionX = 0.0, positionY = 0.0, positionZ = 0.0;
  // double velocityX = 0.0, velocityY = 0.0, velocityZ = 0.0;
  // double timeInterval = 0.0;
  // int num = 0;

  // RobotCarControl rcc(socketfd);
  // rcc.MultiplePoints(pointNum, positionX, positionY, positionZ, 
  //                                         velocityX, velocityY, velocityZ, 
  //                                         timeInterval, num);
  // // int flag;
  // // // cout << "当下爪子张开，是否闭合";
  // // // cin >> flag;
  // // // RobotClaw_.claw_close_control();
  // // cout << "当下爪子闭合，是否张开";
  // // cin >> flag;
  // // RobotClaw_.claw_open_control();
}

int main(int argc, char *argv[])
{
  car_test();

  return 0;
}
