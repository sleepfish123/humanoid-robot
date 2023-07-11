#ifndef ROBOT_CLAW_H__
#define ROBOT_CLAW_H__
#include "big_grey_control/Robot.h"
#define CLAW_OPEN 1
#define CLAW_CLOSE 2
class RobotClaw
{
  private:
  ros::NodeHandle nh;
  public:
  serial::Serial ser_claw;
  int claw_state;      
  void claw_close_msg(uint8_t *close);
  void claw_open_msg(uint8_t *open);
  void claw_close_control();
  void claw_open_control();
  RobotClaw(const std::string serial_name,int baudrate);
  ~RobotClaw();
};

#endif