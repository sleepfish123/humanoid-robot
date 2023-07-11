/*
* 文件名：RobotCarControl.h
* 描述：大灰小车控制代码的头文件
* 修改人：于川
* 修改时间：2022-09-08
* 修改内容：新增

* 修改人：于川
* 修改时间：2022-09-09
* 修改内容：新增三个函数：①车速和轮子转速的转换关系（2个函数） ②判断轮子速度是否越界
*/

#pragma once // 保证头文件只被编译一次

#include"big_grey_control/Robot.h"
#include <serial/serial.h>  

class RobotCarControl {
private:
  /************************************** 宏定义 **************************************/
  // 命令帧格式：帧头(2byte) + 地址(1byte) + 命令(1byte) + 数据(8byte) + CRC校验(2byte)
  // 帧头(2byte)：FB 75
  // 地址(1byte)：默认地址：00

  // 命令帧长度
  #define FRAME_LENGTH                                                        14
  #define MULTIPLE_POINTS_DATA_LENGTH                  22
  // 帧头(2byte)：FB 75
  #define HEAD_1                                   0xFB
  #define HEAD_2                                   0x75
  // 地址(1byte)：默认地址：00
  #define DEFAULT_ADDR                    0x00
  // CRC校验(2byte)
  #define CRC_1                                       0xEF
  #define CRC_2                                       0x5C
  
  // 各命令(1byte)
  #define MULTIPLE_POINTS_DATA_INSTRUCTION                                  0x1b
  #define MULTIPLE_POINTS_PARAMETER_INSTRUCTION                   0x1c

  #define SET_SPEED_INSTRUCTION                                               0x12
  #define OPERATIONG_MODE_INSTRUCTION                             0x12
  #define SET_POSITION_INSTRUCTION                                        0x13
  #define SET_XY_COORDINATES_INSTRUCTION                        0x14
  #define SET_POLAR_COORDINATES_INSTRUCTION               0x15
  #define SET_SPEED_DIRECTION_INSTRUCTION                       0x16
  #define SET_POSITION_DIRECTION_INSTRUCTION                0x17
  #define READ_SPEED_INSTRUCTION                                              0x22
  #define READ_POSITION_INSTRUCTION                                       0x23
  #define READ_XY_COORDINATES_INSTRUCTION                       0x24
  #define READ_SPEED_DIRECTION_INSTRUCTION                    0x26

  #define SPEED_CONVERSION_PARAMETER                                  0.63
  #define MAX_WHEEL_SPEED                                                               1.914                      // 轮子的最大转速 = 400000 / 208986.415 = 1.914 单位是m/s

  #define WIFI_CONTROL        0 // WIFI控制小车
  #define SERIAL_CONTROL  1 // 有线控制小车

  #define CAR_BAUNDRATE     115200

  /******************************************* 变量定义 **************************************/
  struct sockaddr_in addr;
  int m_socketfd;
  int res;
  // ros::NodeHandle nh;

  serial::Serial ser_car;

public:
  /****************************************** 声明各函数及对应功能 ******************************************/
  RobotCarControl();
  RobotCarControl(int socketfd); // 构造函数
  ~RobotCarControl(); // 析构函数
  // 小车多点运行
  void MultiplePointsData(uint8_t* multiplePointsData, int pointNum);
  void MultiplePointsParameter(uint8_t* multiplePointsParameter, double positionX, double positionY, double positionZ, 
                                                                double velocityX, double velocityY, double velocityZ, double timeInterval, int num);
  void MultiplePoints(int pointNum, double positionX, double positionY, double positionZ, 
                                            double velocityX, double velocityY, double velocityZ, double timeInterval, int num);
  // // 设定小车运行模式
  // void RobotCarControl::SetOperatingModeData(uint8_t* operatingModeData, int mode);
  // void RobotCar::SetOperatingMode(int mode);
  // // 设定轮子转动转速
  void SetWheelSpeedData(uint8_t* speedData, int speed);
  void SetSpeed(int speed);

  // 车速和轮子转速的转换关系
  // 车速 =>轮子转速
  Eigen::Vector4d Car_Speed_To_Wheel_Speed(double speedX, double speedY, double speedZ);
  // 轮子转速 => 车速
  Eigen::Vector3d Wheel_Speed_To_Car_Speed(double frontLeftSpeed, double frontRightSpeed, 
                                                                                                                                      double backLeftSpeed, double backRightSpeed);
  // 判断轮子速度是否越界，若越界则返回true
  bool Wheel_Speed_Out(int wheelSpeed);
  // 

  // // 设定轮子转动位置
  // void RobotCarControl::SetPositionData(uint8_t* positionData, int position);
  // void RobotCar::SetPosition(int position);
  // // 设定XY坐标
  // void RobotCarControl::SetXYCoordinatesData(uint8_t* setXYCoordinatesData, int carLeftRight, int carFrontBack, bool enable, int coordinate);
  // void RobotCar::SetXYCoordinates(double length);
  // // 设定小车速度及方向
  // void RobotCarControl::SetSpeedDirectionData(uint8_t* speedDirectionData, uint16_t speed, bool enable, int rotationParameter);
  // void RobotCar::SetSpeedDirection(int speed, bool enable, int rotationParameter);
  // // 设定小车位置及方向
  // void RobotCarControl::SetPositionDirectionData(uint8_t* positionDirectionData, int position, bool enable, int theta);
  // void RobotCar::SetPositionDirection(int position, bool enable, int theta);
  // // 设定小车运行速度
  // void RobotCarControl::SetCarSpeedData(uint8_t* carSpeedData, int speed, int rotationSpeed);
  // void RobotCar::SetCarSpeed(int speed, int rotationSpeed);
  // // 读取转速
  // void RobotCarControl::ReadSpeedData(uint8_t* readSpeedData);
  // void RobotCar::ReadSpeed();
  // // 读取位置
  // void RobotCarControl::ReadPositionData(uint8_t* readPositionData);
  // void RobotCar::ReadPosition();
  // // 读取XY坐标
  // void RobotCarControl::ReadXYCoordinatesData(uint8_t* readXYCoordinatesData);
  // void RobotCar::ReadXYCoordinates();
  // // 读取小车速度及方向
  // void RobotCarControl::ReadSpeedDirectionData(uint8_t* readSpeedDirectionData);
  // void RobotCar::ReadSpeedDirection();
  
  
};



