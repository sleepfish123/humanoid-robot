#ifndef servo_control_H
#define servo_control_H

#include <iostream>
#include <string.h>
#include <std_msgs/String.h>

using namespace  std;
void scs_action_write(uint8_t *action);
void sms_action_write(uint8_t *action);
void scs_regwrite(uint8_t *regwrite_format, int id, float theta, float init_data); // 异步写
void sms_regwrite(uint8_t *regwrite_format, int id, float theta, float init_data); // 异步写
size_t scs_control_servo(int id, float theta, uint8_t* control_msg, uint8_t* action_msg, float init_data); // 给角度值，换算成弧度值，配置控制信号，向串口写入数据
size_t sms_control_servo(int id, float theta, uint8_t* control_msg, uint8_t* action_msg, float init_data); // 给角度值，换算成弧度值，配置控制信号，向串口写入数据

#endif