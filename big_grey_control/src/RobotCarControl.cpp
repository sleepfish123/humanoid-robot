/*
* 文件名：RobotCarControl.cpp
* 描述：大灰小车控制代码
* 修改人：于川
* 修改时间：2022-09-08
* 修改内容：新增

* 修改人：于川
* 修改时间：2022-09-09
* 修改内容：新增三个函数：①车速和轮子转速的转换关系（2个函数） ②判断轮子速度是否越界
*/

#include<big_grey_control/RobotCarControl.h>

// 构造函数
RobotCarControl::RobotCarControl() {
  setlocale(LC_ALL,"");
  ser_car.setPort(CAR_SERIAL_DEFAULT_NAME);    //打开控制机械臂的串口
  ser_car.setBaudrate(CAR_BAUNDRATE);
  if(ser_car.isOpen())
  {
      ROS_INFO("***小车串口已打开,无需重复打开***");
  }
  else
  {
      try
      {
          serial::Timeout to=serial::Timeout::simpleTimeout(1000);
          ser_car.setTimeout(to);
          ser_car.open();
      }
      catch(serial::IOException &e)
      {
          ROS_INFO("***不能打开小车串口!***");
          sleep(10); // *********************
          exit(-1);
      }
  }
  if(ser_car.isOpen())
  {
      ROS_INFO("***小车串口已经打开***");
  }
  else
  {
      ROS_INFO("***小车串口未打开***");
      sleep(10); // *********************
  }
}

// 构造函数
RobotCarControl::RobotCarControl(int socketfd) {
  setlocale(LC_ALL,"");
  m_socketfd = socketfd;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(Port);
  addr.sin_addr.s_addr = inet_addr(ServerIP);
  res = connect(m_socketfd, (struct sockaddr*)&addr, sizeof(addr));

  if (res == -1) {
    ROS_INFO("***小车客户端连接失败***");
  } else {
    ROS_INFO("***小车客户端连接成功***");
  }
}

// 析构函数
RobotCarControl::~RobotCarControl() {
  setlocale(LC_ALL,"");
  ROS_INFO("***小车析构函数被调用***");
  close(m_socketfd); // 关闭客户端套接字
}


/**************************** 小车多点运行 ****************************/
/****** 设定小车多点运行命令 ********/
void RobotCarControl::MultiplePointsData(uint8_t* multiplePointsData, int pointNum) {
  // 帧头(2byte)：FB 75
  multiplePointsData[0] = HEAD_1;
  multiplePointsData[1] = HEAD_2;
  // 地址(1byte)：默认地址：00
  multiplePointsData[2] = DEFAULT_ADDR;
  // 命令(1byte)
  multiplePointsData[3] = MULTIPLE_POINTS_DATA_INSTRUCTION;
  // 数据(8byte)
  multiplePointsData[4] = pointNum;
  multiplePointsData[5] = 0x00;
  multiplePointsData[6] = 0x00;
  multiplePointsData[7] = 0x00;
  multiplePointsData[8] = 0x00;
  multiplePointsData[9] = 0x00;
  multiplePointsData[10] = 0x00;
  multiplePointsData[11] = 0x00;
  // CRC校验(2byte)
  multiplePointsData[12] = CRC_1;
  multiplePointsData[13] = CRC_2;
}

/****** 设定小车多点运行参数 ********/
void RobotCarControl::MultiplePointsParameter(uint8_t* multiplePointsParameter, double positionX, double positionY, double positionZ, 
                                                                                                      double velocityX, double velocityY, double velocityZ, double timeInterval, int num) {
  // 帧头(2byte)：FB 75
  multiplePointsParameter[0] = HEAD_1;
  multiplePointsParameter[1] = HEAD_2;
  // 地址(1byte)：默认地址：00
  multiplePointsParameter[2] = DEFAULT_ADDR;
  // 命令(1byte)
  multiplePointsParameter[3] = MULTIPLE_POINTS_PARAMETER_INSTRUCTION;
  // 数据(8byte)
  // 位置X
  multiplePointsParameter[4] = (int)positionX >> 8; // 高字节
  multiplePointsParameter[5] = (int)positionX & 0xFF; // 低字节
  // 位置Y
  multiplePointsParameter[6] = (int)positionY >> 8; // 高字节
  multiplePointsParameter[7] = (int)positionY & 0xFF; // 低字节
  // 位置Z
  multiplePointsParameter[8] = (int)positionZ >> 8; // 高字节
  multiplePointsParameter[9] = (int)positionZ & 0xFF; // 低字节
  // 速度X
  multiplePointsParameter[10] = (int)velocityX >> 8; // 高字节
  multiplePointsParameter[11] = (int)velocityX & 0xFF; // 低字节
  // 速度Y
  multiplePointsParameter[12] = (int)velocityY >> 8; // 高字节
  multiplePointsParameter[13] = (int)velocityY & 0xFF; // 低字节
  // 速度Z
  multiplePointsParameter[14] = (int)velocityZ >> 8; // 高字节
  multiplePointsParameter[15] = (int)velocityZ & 0xFF; // 低字节
  // 时间
  multiplePointsParameter[16] = (int)timeInterval >> 8; // 高字节
  multiplePointsParameter[17] = (int)timeInterval & 0xFF; // 低字节
  // 点的序列号
  multiplePointsParameter[18] = num >> 8; // 高字节
  multiplePointsParameter[19] = num & 0xFF; // 低字节
  // CRC校验(2byte)
  multiplePointsParameter[20] = CRC_1;
  multiplePointsParameter[21] = CRC_2;
}

/****** 控制代码 ********/
void RobotCarControl::MultiplePoints(int pointNum, double positionX, double positionY, double positionZ, 
                                                                                double velocityX, double velocityY, double velocityZ, double timeInterval, int num) {

  // 车速转换为轮速
  Eigen::Vector4d wheelSpeed;
  wheelSpeed = Car_Speed_To_Wheel_Speed(velocityX, velocityY, velocityZ);

  // 如果轮子转速越界，则直接返回
  for (int i = 0; i < 4; i++) {
    cout << "wheelSpeed " << i << ": " << wheelSpeed[i] << endl;
    if (Wheel_Speed_Out(wheelSpeed[i])) {
      ROS_INFO("***错误：轮子转速越界***");
      return;
    }
  }

  ros::Time::init();

  uint8_t multiplePointsData[FRAME_LENGTH] = {0};
  MultiplePointsData(multiplePointsData, pointNum);
  if (res >= 0) {
#if WIFI_CONTROL
    write(m_socketfd, multiplePointsData, FRAME_LENGTH);
#elif SERIAL_CONTROL
    ser_car.flush();
    ser_car.write(multiplePointsData, FRAME_LENGTH);
#endif
    // usleep(25000); // 延时25ms
    usleep(100000);
  } else {
    ROS_INFO("***设置【设定小车多点运行命令】失败，小车客户端未成功连接***");
  }
  // 打印串口数据
  cout << "打印串口数据：" << endl;
  for (int i = 0; i < FRAME_LENGTH; i++) {
    printf("%x\t", multiplePointsData[i]);
  }
  cout << endl;

  uint8_t multiplePointsParameter[MULTIPLE_POINTS_DATA_LENGTH] = {0}; // 命令帧的长度为22
  MultiplePointsParameter(multiplePointsParameter, positionX * 1000, positionY * 1000, positionZ * 1000, 
                                                        velocityX * 1000, velocityY * 1000, velocityZ * 1000, timeInterval * 1000, num);
  if (res >= 0) {
#if WIFI_CONTROL
    write(m_socketfd, multiplePointsParameter, 22);
#elif SERIAL_CONTROL
    ser_car.flush();
    ser_car.write(multiplePointsParameter, 22);
#endif
    // usleep(25000); // 延时25ms
    usleep(100000);
  } else {
    ROS_INFO("***设置【设定小车多点运行参数】失败，小车客户端未成功连接***");
  }

  // 打印串口数据
  cout << "打印串口数据：" << endl;
  for (int i = 0; i < MULTIPLE_POINTS_DATA_LENGTH; i++) {
    printf("%x\t", multiplePointsParameter[i]);
  }
  cout << endl;
}
/**************************************************************************/



// /**************************** 设定小车运行模式 ****************************/
// /****** 设定命令帧格式 ********/
// void RobotCarControl::SetOperatingModeData(uint8_t* operatingModeData, int mode) {
//   // 帧头(2byte)：FB 75
//   operatingModeData[0] = HEAD_1;
//   operatingModeData[1] = HEAD_2;
//   // 地址(1byte)：默认地址：00
//   operatingModeData[2] = DEFAULT_ADDR;
//   // 命令(1byte)
//   operatingModeData[3] = OPERATIONG_MODE_INSTRUCTION;
//   // 数据(8byte)
//   operatingModeData[4] = mode; // 0x11 0x21 0x31 0x41
//   operatingModeData[5] = 0x00;
//   operatingModeData[6] = 0x00;
//   operatingModeData[7] = 0x00;
//   operatingModeData[8] = 0x00;
//   operatingModeData[9] = 0x00;
//   operatingModeData[10] = 0x00;
//   operatingModeData[11] = 0x00;
//   // CRC校验(2byte)
//   operatingModeData[12] = CRC_1;
//   operatingModeData[13] = CRC_2;
// }

// /****** 控制代码 ********/
// void RobotCar::SetOperatingMode(int mode) {
//   ros::Time::init();

//   uint8_t operatingModeData[FRAME_LENGTH] = {0};
//   SetOperatingModeData(operatingModeData);

//   if (res >= 0) {
//     write(m_socketfd, operatingModeData, FRAME_LENGTH);
//     usleep(25000); // 延时25ms
//   } else {
//     ROS_INFO("***设置【设定小车运行模式】失败，小车客户端未成功连接***");
//   }
// }
// /**************************************************************************/


/**************************** 车速和轮子转速的转换关系 ****************************/
/****** 车速 =>轮子转速 ********/
// X正负对应小车左右，Y正负对应小车前后
Eigen::Vector4d RobotCarControl::Car_Speed_To_Wheel_Speed(double speedX, double speedY, double speedZ) {

  double frontLeftSpeed = (speedX - speedY - speedZ * SPEED_CONVERSION_PARAMETER);              // 前 左
  double frontRightSpeed = (speedX + speedY + speedZ * SPEED_CONVERSION_PARAMETER);         // 前 右
  double backLeftSpeed = (speedX + speedY - speedZ * SPEED_CONVERSION_PARAMETER);             // 后 左
  double backRightSpeed = (speedX - speedY + speedZ * SPEED_CONVERSION_PARAMETER);          // 后 右

  Eigen::Vector4d wheelSpeed;
  wheelSpeed << frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed;
  
  return wheelSpeed;
}

/****** 轮子转速 => 车速 ********/
Eigen::Vector3d RobotCarControl::Wheel_Speed_To_Car_Speed(double frontLeftSpeed, double frontRightSpeed, 
                                                                                                                                      double backLeftSpeed, double backRightSpeed) {
  // 上述 车速 =>轮子转速 的公式换算过来的
  double speedX = (frontLeftSpeed + frontRightSpeed) / 20;
  double speedY = (backLeftSpeed - frontLeftSpeed) / 20;
  double speedZ = (frontRightSpeed - backLeftSpeed) / 12.6;

  Eigen::Vector3d carSpeed;
  carSpeed << speedX, speedY, speedZ;

  return carSpeed;
}
/**************************************************************************/



/**************************** 判断轮子速度是否越界 ****************************/
/****** 判断轮子速度是否越界，若越界则返回true ********/
bool RobotCarControl::Wheel_Speed_Out(int wheelSpeed) {
  if (wheelSpeed <= MAX_WHEEL_SPEED && wheelSpeed >= -MAX_WHEEL_SPEED) { // 没越界
    return false;
  }
  return true;
}

// /****** 判断车的速度是否越界，若越界则返回true ********/
// bool RobotCarControl::Car_Speed_Out(int carSpeed) {
//   if (carSpeed < Wheel_Speed_To_Car_Speed(MAX_WHEEL_SPEED)) { // 没越界
//     return false;
//   }
//   return true;
// }
/**************************************************************************/





/**************************** 设定轮子转动转速 ****************************/
/****** 设定命令帧格式 ********/
void RobotCarControl::SetWheelSpeedData(uint8_t* speedData, int speed) {
  // 帧头(2byte)：FB 75
  speedData[0] = 0xFB;
  speedData[1] = 0x75;
  // 地址(1byte)：默认地址：00
  speedData[2] = 0x00;
  // 命令(1byte)
  speedData[3] = 0x12;
  
  // 数据(8byte)

#if 1
  speedData[4] = 0x0F; // 速度高字节   20000
  speedData[5] = 0xA0; // 速度低字节
  speedData[6] = 0x0F; // 速度高字节
  speedData[7] = 0xA0; // 速度低字节
  speedData[8] = 0x0F; // 速度高字节
  speedData[9] = 0xA0; // 速度低字节
  speedData[10] = 0x0F; // 速度高字节
  speedData[11] = 0xA0; // 速度低字节
#else 
    speedData[4] = speed >> 8; // 速度高字节
  speedData[5] = speed & 0xFF; // 速度低字节
  speedData[6] = speed >> 8; // 速度高字节
  speedData[7] = speed & 0xFF; // 速度低字节
  speedData[8] = speed >> 8; // 速度高字节
  speedData[9] = speed & 0xFF; // 速度低字节
  speedData[10] = speed >> 8; // 速度高字节
  speedData[11] = speed & 0xFF; // 速度低字节
#endif

  // CRC校验(2byte)
  speedData[12] = 0xef;
  speedData[13] = 0x5c;
}

/****** 控制代码 ********/
void RobotCarControl::SetSpeed(int wheelSpeed) {
  
  // 如果轮子转速越界，则直接返回
  if (Wheel_Speed_Out(wheelSpeed)) {
    ROS_INFO("***错误：轮子转速越界***");
    return;
  }
  
  ros::Time::init();
  uint8_t speedData[FRAME_LENGTH] = {0};
  SetWheelSpeedData(speedData, wheelSpeed);

  if (res >= 0) {
    write(m_socketfd, speedData, FRAME_LENGTH);
    usleep(25000); // 延时25ms
  } else {
    ROS_INFO("***设置【设定轮子转动转速】失败，小车客户端未成功连接***");
  }
}
/**************************************************************************/


// /**************************** 设定车速 ****************************/
// /****** 设定命令帧格式 ********/

// /****** 控制代码 ********/
// void RobotCarControl::SetCarSpeed(int speedX, int speedY, int speedZ) {
  
//   Eigesn::Vector4d carSpeedData;
//   carSpeedData << speedX, speedY, speedZ;

//   // 车速转换为轮速
//   Eigesn::Vector4d wheelSpeed;
//   wheelSpeed = Car_Speed_To_Wheel_Speed(carSpeedData);

//   // 如果轮子转速越界，则直接返回
//   for (int i = 0; i < 4; i++) {
//     if (Wheel_Speed_Out(wheelSpeed[i])) {
//       ROS_INFO("***错误：轮子转速越界***");
//       return;
//     }
//   }
  
//   ros::Time::init();

//   uint8_t frontLeftSpeedData[FRAME_LENGTH] = {0}; // 前 左
//   uint8_t frontRightSpeedData[FRAME_LENGTH] = {0}; // 前 右
//   uint8_t backLeftSpeedData[FRAME_LENGTH] = {0}; // 后 左
//   uint8_t backRightSpeedData[FRAME_LENGTH] = {0}; // 后 右

//   SetWheelSpeedData(frontLeftSpeedData, wheelSpeed[0]);
//   SetWheelSpeedData(frontRightSpeedData, wheelSpeed[1]);
//   SetWheelSpeedData(backLeftSpeedData, wheelSpeed[2]);
//   SetWheelSpeedData(backRightSpeedData, wheelSpeed[3]);

//   if (res >= 0) {
//     write(m_socketfd, frontLeftSpeedData, FRAME_LENGTH);
//     usleep(25000); // 延时25ms
//     write(m_socketfd, frontRightSpeedData, FRAME_LENGTH);
//     usleep(25000); // 延时25ms
//     write(m_socketfd, backLeftSpeedData, FRAME_LENGTH);
//     usleep(25000); // 延时25ms
//     write(m_socketfd, backRightSpeedData, FRAME_LENGTH);
//     usleep(25000); // 延时25ms
//   } else {
//     ROS_INFO("***设置【设定轮子转动转速】失败，小车客户端未成功连接***");
//   }
// }
/**************************************************************************/




// /**************************** 设定轮子转动位置 ****************************/
// /****** 设定命令帧格式 ********/
// void RobotCarControl::SetPositionData(uint8_t* positionData, int position) {
//   // 帧头(2byte)：FB 75
//   positionData[0] = HEAD_1;
//   positionData[1] = HEAD_2;
//   // 地址(1byte)：默认地址：00
//   positionData[2] = DEFAULT_ADDR;
//   // 命令(1byte)
//   positionData[3] = SET_POSITION_INSTRUCTION;
//   // 数据(8byte)
//   positionData[4] = position >> 8; // 速度高字节
//   positionData[5] = position & 0xFF; // 速度低字节
//   positionData[6] = position >> 8; // 速度高字节
//   positionData[7] = position & 0xFF; // 速度低字节
//   positionData[8] = position >> 8; // 速度高字节
//   positionData[9] = position & 0xFF; // 速度低字节
//   positionData[10] = position >> 8; // 速度高字节
//   positionData[11] = position & 0xFF; // 速度低字节
//   // CRC校验(2byte)
//   positionData[12] = CRC_1;
//   positionData[13] = CRC_2;
// }

// /****** 控制代码 ********/
// void RobotCar::SetPosition(int position) {
//   ros::Time::init();

//   uint8_t positionData[FRAME_LENGTH] = {0};
//   SetPositionData(positionData);

//   if (res >= 0) {
//     write(m_socketfd, positionData, FRAME_LENGTH);
//     usleep(25000); // 延时25ms
//   } else {
//     ROS_INFO("***设置【设定轮子转动位置】失败，小车客户端未成功连接***");
//   }
// }
// /**************************************************************************/


// /**************************** 设定XY坐标 ****************************/
// /****** 设定命令帧格式 ********/
// void RobotCarControl::SetXYCoordinatesData(uint8_t* setXYCoordinatesData, int carLeftRight, int carFrontBack, bool enable, int coordinate) {
//   // 帧头(2byte)：FB 75
//   setXYCoordinatesData[0] = HEAD_1;
//   setXYCoordinatesData[1] = HEAD_2;
//   // 地址(1byte)：默认地址：00
//   setXYCoordinatesData[2] = DEFAULT_ADDR;
//   // 命令(1byte)
//   setXYCoordinatesData[3] = SET_XY_COORDINATES_INSTRUCTION;
//   // 数据(8byte)
//   // X坐标，为0则停止
//   setXYCoordinatesData[4] = carLeftRight >> 8; // 高字节
//   setXYCoordinatesData[5] = carLeftRight & 0xFF; // 低字节
//   // Y坐标，为0则停止
//   setXYCoordinatesData[6] = carFrontBack >> 8; // 高字节
//   setXYCoordinatesData[7] = carFrontBack & 0xFF; // 低字节
  
//   setXYCoordinatesData[8] = enable; // 使能
//   setXYCoordinatesData[9] = coordinate; // 坐标系
//   // 其余两个字节为0
//   setXYCoordinatesData[10] = 0; 
//   setXYCoordinatesData[11] = 0; 
//   // CRC校验(2byte)
//   setXYCoordinatesData[12] = CRC_1;
//   setXYCoordinatesData[13] = CRC_2;
// }
// /****** 控制代码 ********/
// void RobotCar::SetXYCoordinates(double length) {
//   ros::Time::init();

//   uint8_t setXYCoordinatesData[FRAME_LENGTH] = {0};
//   SetXYCoordinatesData(setXYCoordinatesData);

//   if (res >= 0) {
//     write(m_socketfd, setXYCoordinatesData, FRAME_LENGTH);
//     usleep(25000); // 延时25ms
//   } else {
//     ROS_INFO("***设置【设定XY坐标】失败，小车客户端未成功连接***");
//   }
// }
// /**************************************************************************/


// /**************************** 设定小车速度及方向 ****************************/
// /****** 设定命令帧格式 ********/
// void RobotCarControl::SetSpeedDirectionData(uint8_t* speedDirectionData, uint16_t speed, bool enable, int rotationParameter) {
//   // 帧头(2byte)：FB 75
//   speedDirectionData[0] = HEAD_1;
//   speedDirectionData[1] = HEAD_2;
//   // 地址(1byte)：默认地址：00
//   speedDirectionData[2] = DEFAULT_ADDR;
//   // 命令(1byte)
//   speedDirectionData[3] = SET_SPEED_DIRECTION_INSTRUCTION;
//   // 数据(8byte)
//   // 速度 + 方向？
//   speedDirectionData[4] = speed >> 8; // 速度
//   speedDirectionData[5] = speed & 0xFF; // 速度
//   speedDirectionData[6] =  >> 8; // 方向
//   speedDirectionData[7] =  & 0xFF; // 方向
//   speedDirectionData[8] = enable; // 使能
//   speedDirectionData[9] = rotationParameter; // 旋转参数
//   // 小车旋转速度或角度
//   speedDirectionData[10] = 0; // 高字节
//   speedDirectionData[11] = 0; // 低字节
//   // CRC校验(2byte)
//   speedDirectionData[12] = CRC_1;
//   speedDirectionData[13] = CRC_2;
// }
// /****** 控制代码 ********/
// void RobotCar::SetSpeedDirection(int speed, bool enable, int rotationParameter) {
//   ros::Time::init();

//   uint8_t speedDirectionData[FRAME_LENGTH] = {0};
//   SetSpeedDirectionData(speedDirectionData, speed, enable, rotationParameter);

//   if (res >= 0) {
//     write(m_socketfd, speedDirectionData, FRAME_LENGTH);
//     usleep(25000); // 延时25ms
//   } else {
//     ROS_INFO("***设置【设定小车速度及方向】失败，小车客户端未成功连接***");
//   }
// }
// /**************************************************************************/


// /**************************** 设定小车位置及方向 ****************************/
// /****** 设定命令帧格式 ********/
// void RobotCarControl::SetPositionDirectionData(uint8_t* positionDirectionData, int position, bool enable, int theta) {
//   // 帧头(2byte)：FB 75
//   positionDirectionData[0] = HEAD_1;
//   positionDirectionData[1] = HEAD_2;
//   // 地址(1byte)：默认地址：00
//   positionDirectionData[2] = DEFAULT_ADDR;
//   // 命令(1byte)
//   positionDirectionData[3] = SET_POSITION_DIRECTION_INSTRUCTION;
//   // 数据(8byte)
//   positionDirectionData[4] = position >> 8; // 位置
//   positionDirectionData[5] = position & 0xFF; // 位置
//   positionDirectionData[6] =  >> 8; // 方向
//   positionDirectionData[7] =  & 0xFF; // 方向
//   positionDirectionData[8] = enable; // 使能
//   // 小车旋转角度
//   positionDirectionData[9] = theta >> 8; // 高字节
//   positionDirectionData[10] = theta & 0xFF; // 低字节

//   positionDirectionData[11] = 0;
//   // CRC校验(2byte)
//   positionDirectionData[12] = CRC_1;
//   positionDirectionData[13] = CRC_2;
// }

// /****** 控制代码 ********/
// void RobotCar::SetPositionDirection(int position, bool enable, int theta) {
//   ros::Time::init();

//   uint8_t positionDirectionData[FRAME_LENGTH] = {0};
//   SetPositionDirectionData(positionDirectionData, position, enable, theta);

//   if (res >= 0) {
//     write(m_socketfd, positionDirectionData, FRAME_LENGTH);
//     usleep(25000); // 延时25ms
//   } else {
//     ROS_INFO("***设置【设定小车位置及方向】失败，小车客户端未成功连接***");
//   }
// }
// /**************************************************************************/


// /**************************** 设定小车运行速度 ****************************/
// /****** 设定命令帧格式 ********/
// void RobotCarControl::SetCarSpeedData(uint8_t* carSpeedData, int speed, int rotationSpeed) {
//   // 帧头(2byte)：FB 75
//   carSpeedData[0] = HEAD_1;
//   carSpeedData[1] = HEAD_2;
//   // 地址(1byte)：默认地址：00
//   carSpeedData[2] = DEFAULT_ADDR;
//   // 命令(1byte)
//   carSpeedData[3] = SET_CAR_SPEED_INSTRUCTION;
//   // 数据(8byte)
//   // 运行速度
//   carSpeedData[4] = speed >> 8; // 高字节
//   carSpeedData[5] = speed & 0xFF; // 低字节
//   // 旋转速度
//   carSpeedData[6] = rotationSpeed >> 8; // 高字节
//   carSpeedData[7] = rotationSpeed & 0xFF; // 低字节
//   // 其余均为0
//   carSpeedData[8] = 0;
//   carSpeedData[9] = 0;
//   carSpeedData[10] = 0;
//   carSpeedData[11] = 0;
//   // CRC校验(2byte)
//   carSpeedData[12] = CRC_1;
//   carSpeedData[13] = CRC_2;
// }
// /****** 控制代码 ********/
// void RobotCar::SetCarSpeed(int speed, int rotationSpeed) {
//   ros::Time::init();

//   uint8_t carSpeedData[FRAME_LENGTH] = {0};
//   SetCarSpeedData(carSpeedData, position, enable, theta);

//   if (res >= 0) {
//     write(m_socketfd, carSpeedData, FRAME_LENGTH);
//     usleep(25000); // 延时25ms
//   } else {
//     ROS_INFO("***设置【设定小车运行速度】失败，小车客户端未成功连接***");
//   }
// }
// /**************************************************************************/


// /**************************** 读取转速 ****************************/
// /****** 设定命令帧格式 ********/
// void RobotCarControl::ReadSpeedData(uint8_t* readSpeedData) {
//   // 帧头(2byte)：FB 75
//   readSpeedData[0] = HEAD_1;
//   readSpeedData[1] = HEAD_2;
//   // 地址(1byte)：默认地址：00
//   readSpeedData[2] = DEFAULT_ADDR;
//   // 命令(1byte)
//   readSpeedData[3] = READ_SPEED_INSTRUCTION;
//   // 数据(8byte)
//   // 8字节均为0
//   readSpeedData[4] = 0;
//   readSpeedData[5] = 0;
//   readSpeedData[6] = 0;
//   readSpeedData[7] = 0;
//   readSpeedData[8] = 0;
//   readSpeedData[9] = 0;
//   readSpeedData[10] = 0;
//   readSpeedData[11] = 0;
//   // CRC校验(2byte)
//   readSpeedData[12] = CRC_1;
//   readSpeedData[13] = CRC_2;
// }
// /****** 控制代码 ********/
// void RobotCar::ReadSpeed() {
//   ros::Time::init();

//   uint8_t readSpeedData[FRAME_LENGTH] = {0};
//   ReadSpeedData(positionDirectionData);

//   if (res >= 0) {
//     write(m_socketfd, readSpeedData, FRAME_LENGTH);
//     usleep(25000); // 延时25ms
//   } else {
//     ROS_INFO("***设置【读取转速】失败，小车客户端未成功连接***");
//   }
// }
// /**************************************************************************/


// /**************************** 读取位置 ****************************/
// /****** 设定命令帧格式 ********/
// void RobotCarControl::ReadPositionData(uint8_t* readPositionData) {
//   // 帧头(2byte)：FB 75
//   readPositionData[0] = HEAD_1;
//   readPositionData[1] = HEAD_2;
//   // 地址(1byte)：默认地址：00
//   readPositionData[2] = DEFAULT_ADDR;
//   // 命令(1byte)
//   readPositionData[3] = READ_POSITION_INSTRUCTION;
//   // 数据(8byte)
//   // 8字节均为0
//   readPositionData[4] = 0;
//   readPositionData[5] = 0;
//   readPositionData[6] = 0;
//   readPositionData[7] = 0;
//   readPositionData[8] = 0;
//   readPositionData[9] = 0;
//   readPositionData[10] = 0;
//   readPositionData[11] = 0;
//   // CRC校验(2byte)
//   readPositionData[12] = CRC_1;
//   readPositionData[13] = CRC_2;
// }

// /****** 控制代码 ********/
// void RobotCar::ReadPosition() {
//   ros::Time::init();

//   uint8_t readPositionData[FRAME_LENGTH] = {0};
//   ReadPositionData(readPositionData);

//   if (res >= 0) {
//     write(m_socketfd, readPositionData, FRAME_LENGTH);
//     usleep(25000); // 延时25ms
//   } else {
//     ROS_INFO("***设置【读取位置】失败，小车客户端未成功连接***");
//   }
// }
// /**************************************************************************/


// /**************************** 读取XY坐标 ****************************/
// /****** 设定命令帧格式 ********/
// void RobotCarControl::ReadXYCoordinatesData(uint8_t* readXYCoordinatesData) {
//   // 帧头(2byte)：FB 75
//   readXYCoordinatesData[0] = HEAD_1;
//   readXYCoordinatesData[1] = HEAD_2;
//   // 地址(1byte)：默认地址：00
//   readXYCoordinatesData[2] = DEFAULT_ADDR;
//   // 命令(1byte)
//   readXYCoordinatesData[3] = READ_XY_COORDINATES_INSTRUCTION;
//   // 数据(8byte)
//   // 8字节均为0
//   readPositionData[4] = 0;
//   readPositionData[5] = 0;
//   readPositionData[6] = 0;
//   readPositionData[7] = 0;
//   readPositionData[8] = 0;
//   readPositionData[9] = 0;
//   readPositionData[10] = 0;
//   readPositionData[11] = 0;
//   // CRC校验(2byte)
//   readPositionData[12] = CRC_1;
//   readPositionData[13] = CRC_2;
// }

// /****** 控制代码 ********/
// void RobotCar::ReadXYCoordinates() {
//   ros::Time::init();

//   uint8_t readXYCoordinatesData[FRAME_LENGTH] = {0};
//   ReadXYCoordinatesData(readXYCoordinatesData);

//   if (res >= 0) {
//     write(m_socketfd, readXYCoordinatesData, FRAME_LENGTH);
//     usleep(25000); // 延时25ms
//   } else {
//     ROS_INFO("***设置【读取XY坐标】失败，小车客户端未成功连接***");
//   }
// }
// /**************************************************************************/


// /**************************** 读取小车速度及方向 ****************************/
// /****** 设定命令帧格式 ********/
// void RobotCarControl::ReadSpeedDirectionData(uint8_t* readSpeedDirectionData) {
//   // 帧头(2byte)：FB 75
//   readSpeedDirectionData[0] = HEAD_1;
//   readSpeedDirectionData[1] = HEAD_2;
//   // 地址(1byte)：默认地址：00
//   readSpeedDirectionData[2] = DEFAULT_ADDR;
//   // 命令(1byte)
//   readSpeedDirectionData[3] = READ_SPEED_DIRECTION_INSTRUCTION;
//   // 数据(8byte)
//   // 8字节均为0
//   readSpeedDirectionData[4] = 0;
//   readSpeedDirectionData[5] = 0;
//   readSpeedDirectionData[6] = 0;
//   readSpeedDirectionData[7] = 0;
//   readSpeedDirectionData[8] = 0;
//   readSpeedDirectionData[9] = 0;
//   readSpeedDirectionData[10] = 0;
//   readSpeedDirectionData[11] = 0;
//   // CRC校验(2byte)
//   readSpeedDirectionData[12] = CRC_1;
//   readSpeedDirectionData[13] = CRC_2;
// }
// /****** 控制代码 ********/
// void RobotCar::ReadSpeedDirection() {
//   ros::Time::init();

//   uint8_t readSpeedDirectionData[FRAME_LENGTH] = {0};
//   ReadSpeedDirectionData(readSpeedDirectionData);

//   if (res >= 0) {
//     write(m_socketfd, readSpeedDirectionData, FRAME_LENGTH);
//     usleep(25000); // 延时25ms
//   } else {
//     ROS_INFO("***设置【读取小车速度及方向】失败，小车客户端未成功连接***");
//   }
// }
// /**************************************************************************/



// // 设定极坐标
// // 设定当前位置为指定坐标点
// // 设定小车多点运行命令
// // 设定小车多点运行参数
// // 读取电池信息
// // 读取极坐标
// // 读取当前节电模式
// // 读取当前小车数据
// // 读取当前电池电量
// // 读取当前超声波数据




// void RobotCar::car_stop_control()
// {
//   ros::Time::init();
//   ros::Rate loop_rate(100);
//   setlocale(LC_ALL,"");
//   uint8_t car_stop[FRAME_LENGTH]={0};
//   car_stop_msg(car_stop);
//   if(res>=0)
//   {
//     write(user_socket_fd,car_stop,data_length);
//     loop_rate.sleep();
//     write(user_socket_fd,car_stop,data_length);
//     loop_rate.sleep();
//     write(user_socket_fd,car_stop,data_length);
//     loop_rate.sleep();
//   }
//   else
//   {
//     ROS_INFO("小车客户端未成功连接，无法发送停止信息");
//   }
// }
// void RobotCar::car_forward_control(double length)
// {
//   length=abs(length);
//   ros::Time::init();
//   ros::Rate loop_rate(100);
//   int run_time=length*100/95.7;
//   uint8_t car_run_data[14]={0};
//   car_forward_msg(car_run_data);
//   write(user_socket_fd,car_run_data,data_length);
//   // int b=0;
//   // for(int r=0;r<run_time;r++)
//   // {
//   //   b=r%5;
//   //   if(b==0)
//   //   {
//   //     if(res>=0)
//   //   {
//   //     write(user_socket_fd,car_run_data,data_length);
//   //   } 
//   //    else
//   //   {
//   //     ROS_INFO("小车客户端未成功连接，无法发送前进信息");
//   //   }

//   //   }
//   //   loop_rate.sleep();
//   // }
//   // car_stop_control();
// }

// void RobotCar::car_back_control(double length)
// {
//   ros::Time::init();
//   length=abs(length);
//   ros::Rate loop_rate(100);
//   int run_time=length*100/95.7;
//   uint8_t car_run_data[14]={0};
//   car_back_msg(car_run_data);
//   int b=0;
//   for(int r=0;r<run_time;r++)
//   {
//     b=r%5;
//     if(b==0)
//     {
//       if(res>=0)
//     {
//       write(user_socket_fd,car_run_data,data_length);
//     } 
//      else
//     {
//       ROS_INFO("小车客户端未成功连接，无法发送后退信息");
//     }
//     }
//     loop_rate.sleep();
//   }
//   car_stop_control();
// }

// void RobotCar::car_left_control(double length)
// {
//   ros::Time::init();
//   length=abs(length);
//   ros::Rate loop_rate(100);
//   int run_time=length*100/95.7;
//   uint8_t car_run_data[14]={0};
//   car_left_msg(car_run_data);
//   int b=0;
//   for(int r=0;r<run_time;r++)
//   {
//     b=r%5;
//     if(b==0)
//     {
//       if(res>=0)
//     {
//       write(user_socket_fd,car_run_data,data_length);
//     } 
//      else
//     {
//       ROS_INFO("小车客户端未成功连接，无法发送左移信息");
//     }
//     }
//     loop_rate.sleep();
//   }
//   car_stop_control();
// }

// void RobotCar::car_right_control(double length)
// {
//   ros::Time::init();
//   length=abs(length);
//   ros::Rate loop_rate(100);
//   int run_time=length*100/95.7;
//   uint8_t car_run_data[14]={0};
//   car_right_msg(car_run_data);
//   int b=0;
//   for(int r=0;r<run_time;r++)
//   {
//     b=r%5;
//     if(b==0)
//     {
//       if(res>=0)
//     {
//       write(user_socket_fd,car_run_data,data_length);
//     } 
//      else
//     {
//       ROS_INFO("小车客户端未成功连接，无法发送右移信息");
//     }
//     }
//     loop_rate.sleep();
//   }
//   car_stop_control();
// }

// void RobotCar::car_action(double dx,double dy)
// {
//   if(dx>0)
//   {
//     car_forward_control(dx);
//   }
//   else
//   {
//     car_back_control(dx);
//   }
//   if(dy>0)
//   {
//     car_left_control(dy);
//   }
//   else
//   {
//     car_right_control(dy);
//   }
// }
