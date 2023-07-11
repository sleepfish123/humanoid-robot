#include<big_grey_control/RobotArm.h>
#include<big_grey_control/RobotClaw.h>
using namespace std;

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"robot_test");
  RobotArm RobotArm_(ARM_SERIAL_DEFAULT_NAME, ARM_SERIAL_DEFAULT_BAUDRATE);
  RobotClaw RobotClaw_(CLAW_SERIAL_DEFAULT_NAME,CLAW_SERIAL_DEFAULT_BAUDRATE);

  return 0;
}
