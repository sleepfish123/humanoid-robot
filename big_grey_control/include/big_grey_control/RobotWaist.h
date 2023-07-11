#pragma once
#include"big_grey_control/Robot.h"
class RobotWaist
{
private:
  struct sockaddr_in addr;
  uint8_t foot_data[14];
  uint8_t stop_data[14];
  uint8_t waist_data[14];
  int user_socket_fd;
  size_t length;
  double angle_foot;
  double angle_swing;
  double angle_bend;
  double angle_updown;
  int res;//连接应答标志位
  serial::Serial ser_waist_sensor;
  serial::Serial ser_foot_sensor;
public:
  RobotWaist(int socket_fd,const std::string waist_sensor_serial_name,int waist_sensor_baudrate,
                          const std::string foot_sensor_serial_name,int foot_sensor_baudrate);
  RobotWaist(int socket_fd);
  RobotWaist(int socket_fd,const std::string waist_sensor_serial_name,int waist_sensor_baudrate);
  RobotWaist(int socket_fd,int foot_sensor_baudrate,const std::string foot_sensor_serial_name);
  void waist_up_msg(uint8_t *up,int8_t v1,int8_t v2,int8_t v3);
  void waist_time_msg(uint8_t *data,int8_t l1_action, int8_t l2_action,int8_t l3_action);
  void waist_down_msg(uint8_t *down);
  void stop_msg(uint8_t *stop);
  void foot_msg(uint8_t *foot,int8_t s,uint8_t velocity);
  void foot_waist_action_control(double swing,double bend,double updown,double foot_angle);//角度，角度，毫米，角度
  void foot_action_control(double foot_angle);
  void waist_action_control(double l1,double l2,double l3);//厘米
  void stop_control();
  void read_waist_sensor(double *Gan);//毫米
  double read_foot_sensor();
  void foot_sensor_clear();
  ~RobotWaist();
};


