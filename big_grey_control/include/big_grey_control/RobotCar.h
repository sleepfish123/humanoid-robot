#pragma once
#include"big_grey_control/Robot.h"
class RobotCar
{
private:
  struct sockaddr_in addr;
  int user_socket_fd;
  int res;
  int data_length;
  ros::NodeHandle nh;
public:
  RobotCar(int socket_fd);
  ~RobotCar();
  void car_forward_msg(uint8_t *car_forward_data);
  void car_back_msg(uint8_t *car_back_data);
  void car_left_msg(uint8_t *car_left_data);
  void car_right_msg(uint8_t *car_right_data);
  void car_stop_msg(uint8_t* car_stop_data);

  void car_forward_control(double car_forward_length);
  void car_back_control(double car_back_length);
  void car_left_control(double car_left_length);
  void car_right_control(double car_right_length);
  void car_stop_control();
  
  void car_action(double dx,double dy);//dy大于0左移，dx大于0前移
};



