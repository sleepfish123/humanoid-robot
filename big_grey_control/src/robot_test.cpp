#include<big_grey_control/RobotArm.h>
#include<big_grey_control/RobotClaw.h>
#include<big_grey_control/RobotWaist.h>
#include <big_grey_control/RobotCar.h>
using namespace std;
uint8_t car_run_data[14]={0};

void waist_foot_test()
{
  // 腰部
  int sockfd=socket(PF_INET, SOCK_STREAM,0);
  // RobotWaist RobotWaist_(sockfd,WAIST_SENSOR_SERIAL_DEFAULT_NAME,WAIST_SENSOR_SERIAL_DEFAULT_BAUDRATE);
  RobotWaist RobotWaist_(sockfd, WAIST_SENSOR_SERIAL_DEFAULT_NAME, WAIST_SENSOR_SERIAL_DEFAULT_BAUDRATE,
                          FOOT_SENSOR_SERIAL_DEFAULT_NAME, FOOT_SENSOR_SERIAL_DEFAULT_BAUDRATE);

  double theta_jiaobu = 31.5653; // -4.99
  double theta_ceyao = -2.17686;
  double theta_wanyao = 0.0;
  double up_down_waist = 0.000822504; // m

  // 腰和足
  // RobotWaist_.foot_waist_action_control(theta_ceyao, theta_wanyao, up_down_waist * 1000, theta_jiaobu); // 角度、mm
  // RobotWaist_.foot_waist_action_control(-theta_ceyao, -theta_wanyao, -(up_down_waist + 0.03) * 1000, 0.0);
  RobotWaist_.foot_waist_action_control(-theta_ceyao, -theta_wanyao, -up_down_waist * 1000, 0.0);
  // RobotWaist_.foot_waist_action_control(0.0, 0.0, 0.0, 0.0);
  // int flag;
  // cin >> flag;
  // RobotWaist_.foot_waist_action_control(-theta_ceyao, -theta_wanyao, -up_down_waist * 1000, -theta_jiaobu); // 角度、mm
}

void arm_test()
{
  RobotArm RobotArm_(ARM_SERIAL_DEFAULT_NAME, ARM_SERIAL_DEFAULT_BAUDRATE);
  // RobotArm_.arm_init();
  uint8_t buf[14] = {0};
  memset(buf, 0, 14);
  buf[0] = 0xfb;
  buf[1] = 0x75;
  buf[2] = 0x00;
  buf[3] = 0x03;
  buf[4] = 0x04;
  buf[5] = 0x05;
  buf[6] = 0x06;
  buf[7] = 0x07;
  buf[8] = 0x08;
  buf[9] = 0x08;
  buf[10] = 0x08;
  buf[11] = 0x08;
  buf[12] = 0xef;
  buf[13] = 0x5c;

  int flag = 0;
  // serial::Serial ser_arm;
  RobotArm_.ser_arm.flush();
  RobotArm_.ser_arm.write(buf, 14);
  usleep(100000);

  // while (flag != 9) {
  //   ser_arm.write(buf, 14);
  //   usleep(100000);

  //   // cin >> flag;
  // }
  

  // int flag;
  // cout << "是否让初始化结束，开始动作";
  // cin >> flag;
  // // sleep(2);
  // sensor_msgs::JointState js;
  // js.position.clear();
  // for(int i=0;i<13;i++)
  // {
  //   js.position.push_back(0.0);
  // }
  // /******右的*******/
  // // js.position[0]=0; // 右小臂
  // // js.position[1]=-48.4354+90; // 右肘部v
  // // js.position[2]=-3.467; // 右大臂v
  // // js.position[3]=-2.88428; // 右侧抬，向外抬给负值v
  // // js.position[4]=76.3425; // 右前后抬v
  // // js.position[5]=0; // 点头v-6.34
  // // js.position[6]=0; // 摇头v-15.75

  // /******左的*******/
  // // js.position[7]=10.0;
  // // js.position[8]=-40.0; //向外抬给负值
  // // js.position[9]=-30.0;
  // // js.position[10]=80.0;
  // // js.position[11]=30.0;
  // // js.position[12]=30.0;

  //   /******右的*******/
  // js.position[0]=20; // 右小臂
  // js.position[1]= -35+ 70; // 右肘部v
  // js.position[2]=15.; // 右大臂v
  // js.position[3]=-25; // 右侧抬，向外抬给负值v
  // js.position[4]=80; // 右前后抬v
  // js.position[5]=0; // 点头v-6.34
  // js.position[6]=0; // 摇头v-15.75

  // /******左的*******/
  // js.position[7]=0.0;
  // js.position[8]=0.0; //向外抬给负值
  // js.position[9]=0.0;
  // js.position[10]=0.0;
  // js.position[11]=0.0;
  // js.position[12]=0.0;
  
  // RobotArm_.arm_control(js);
  // cout << "动作结束，是否初始化";
  // cin >> flag;
  // RobotArm_.arm_init();
}

void car_test()
{
  int socket_fd=socket(PF_INET, SOCK_STREAM,0);
  RobotCar RobotCar_(socket_fd);
}

void claw_test()
{
  RobotClaw RobotClaw_(CLAW_SERIAL_DEFAULT_NAME,CLAW_SERIAL_DEFAULT_BAUDRATE);
  int flag;
  // cout << "当下爪子张开，是否闭合";
  // cin >> flag;
  // RobotClaw_.claw_close_control();
  cout << "当下爪子闭合，是否张开";
  cin >> flag;
  RobotClaw_.claw_open_control();
}
int main(int argc, char *argv[])
{
  ros::init(argc,argv,"robot_test");
  arm_test();
  // waist_foot_test();

  // RobotArm RobotArm_(ARM_SERIAL_DEFAULT_NAME, ARM_SERIAL_DEFAULT_BAUDRATE);
  // RobotArm_.arm_init();

  // claw_test();
  return 0;
}
