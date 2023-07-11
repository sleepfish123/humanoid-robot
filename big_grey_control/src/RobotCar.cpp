#include<big_grey_control/RobotCar.h>
RobotCar::RobotCar(int socket_fd)
{
  setlocale(LC_ALL,"");
  user_socket_fd=socket_fd;
  addr.sin_family=AF_INET;
  addr.sin_port=htons(Port);
  addr.sin_addr.s_addr=inet_addr(ServerIP);
  res=connect(socket_fd,(struct sockaddr*)&addr,sizeof(addr));
  if(res==-1)
  {
    ROS_INFO("***小车客户端连接失败***");
  }
  else
  {
    ROS_INFO("***小车客户端连接成功***");
  }
  data_length=14;
}

RobotCar::~RobotCar()
{
  setlocale(LC_ALL,"");
  ROS_INFO("小车析构函数被调用");
  close(user_socket_fd);//关闭客户端套接字
}

void RobotCar::car_forward_msg(uint8_t* car_forward_data)
{
  car_forward_data[0]=0xfb;                    //数据头
  car_forward_data[1]=0x75;                   //数据头
  car_forward_data[2]=0x00;                   //编号位
  car_forward_data[3]=0x12;               //模式位速度模式                //2000 对应实际速度9.57cm/s
  car_forward_data[4]=0x07;                  //前左轮高八位
  car_forward_data[5]=0xd0;                //前左轮低八位
  car_forward_data[6]=0x07;               //前右轮高八位
  car_forward_data[7]=0xd0;                //fr前右轮低八位
  car_forward_data[8]=0x07;                  //后左轮高八位
  car_forward_data[9]=0xd0;               //bl后左轮低八位
  car_forward_data[10]=0x07;             //后右轮高八位
  car_forward_data[11]=0xd0;            //br后右轮低八位
  car_forward_data[12]=0xef;                  //预留CRC校验
  car_forward_data[13]=0x5c;               //预留CRC校验
}

void RobotCar::car_back_msg(uint8_t* car_back_data)
{
  car_back_data[0]=0xfb;                    //数据头
  car_back_data[1]=0x75;                   //数据头
  car_back_data[2]=0x00;                   //编号位
  car_back_data[3]=0x12;               //模式位速度模式                //2000 对应实际速度9.57cm/s
  car_back_data[4]=0xf8;                  //前左轮高八位
  car_back_data[5]=0x30;                //前左轮低八位
  car_back_data[6]=0xf8;               //前右轮高八位
  car_back_data[7]=0x30;                //fr前右轮低八位
  car_back_data[8]=0xf8;                  //后左轮高八位
  car_back_data[9]=0x30;               //bl后左轮低八位
  car_back_data[10]=0xf8;             //后右轮高八位
  car_back_data[11]=0x30;            //br后右轮低八位
  car_back_data[12]=0xef;                  //预留CRC校验
  car_back_data[13]=0x5c;               //预留CRC校验
}

void RobotCar::car_left_msg(uint8_t* car_left_data)
{
  car_left_data[0]=0xfb;
  car_left_data[1]=0x75;                   //数据头
  car_left_data[2]=0x00;                   //编号位
  car_left_data[3]=0x12;               //模式位速度模式                           //2000 对应实际速度8.74cm/s
  car_left_data[4]=0xf8;
  car_left_data[5]=0x30;
  car_left_data[6]=0x07;
  car_left_data[7]=0xd0;
  car_left_data[8]=0x07;                  
  car_left_data[9]=0xd0;
  car_left_data[10]=0xf8;
  car_left_data[11]=0x30;
  car_left_data[12]=0xef;
  car_left_data[13]=0x5c;
}

void RobotCar::car_right_msg(uint8_t* car_right_data)
{
  car_right_data[0]=0xfb;
  car_right_data[1]=0x75;                   //数据头
  car_right_data[2]=0x00;                   //编号位
  car_right_data[3]=0x12;               //模式位速度模式
  car_right_data[4]=0x07;
  car_right_data[5]=0xd0;
  car_right_data[6]=0xf8;
  car_right_data[7]=0x30;
  car_right_data[8]=0xf8;                  
  car_right_data[9]=0x30;
  car_right_data[10]=0x07;
  car_right_data[11]=0xd0;
  car_right_data[12]=0xef;
  car_right_data[13]=0x5c;
}

void RobotCar::car_stop_msg(uint8_t* car_stop_data)
{
  car_stop_data[0]=0xfb;                    //数据头
  car_stop_data[1]=0x75;                   //数据头
  car_stop_data[2]=0x00;                   //编号位
  car_stop_data[3]=0x12;               //模式位速度模式               
  car_stop_data[4]=7000>>8;                  //前左轮高八位
  car_stop_data[5]=7000&0xff;                //前左轮低八位
  car_stop_data[6]=7000>>8;               //前右轮高八位
  car_stop_data[7]=7000&0xff;                //fr前右轮低八位
  car_stop_data[8]=7000>>8;                  //后左轮高八位
  car_stop_data[9]=7000&0xff;               //bl后左轮低八位
  car_stop_data[10]=7000>>8;             //后右轮高八位
  car_stop_data[11]=7000&0xff;            //br后右轮低八位
  car_stop_data[12]=0xef;                  //预留CRC校验
  car_stop_data[13]=0x5c;               //预留CRC校验
}

void RobotCar::car_stop_control()
{
  ros::Time::init();
  ros::Rate loop_rate(100);
  setlocale(LC_ALL,"");
  uint8_t car_stop[14]={0};
  car_stop_msg(car_stop);
  if(res>=0)
  {
    write(user_socket_fd,car_stop,data_length);
    loop_rate.sleep();
    write(user_socket_fd,car_stop,data_length);
    loop_rate.sleep();
    write(user_socket_fd,car_stop,data_length);
    loop_rate.sleep();
  }
  else
  {
    ROS_INFO("小车客户端未成功连接，无法发送停止信息");
  }
}
void RobotCar::car_forward_control(double length)
{
  length=abs(length);
  ros::Time::init();
  ros::Rate loop_rate(100);
  int run_time=length*100/95.7;
  uint8_t car_run_data[14]={0};
  car_forward_msg(car_run_data);
  write(user_socket_fd,car_run_data,data_length);
  // int b=0;
  // for(int r=0;r<run_time;r++)
  // {
  //   b=r%5;
  //   if(b==0)
  //   {
  //     if(res>=0)
  //   {
  //     write(user_socket_fd,car_run_data,data_length);
  //   } 
  //    else
  //   {
  //     ROS_INFO("小车客户端未成功连接，无法发送前进信息");
  //   }

  //   }
  //   loop_rate.sleep();
  // }
  // car_stop_control();
}

void RobotCar::car_back_control(double length)
{
  ros::Time::init();
  length=abs(length);
  ros::Rate loop_rate(100);
  int run_time=length*100/95.7;
  uint8_t car_run_data[14]={0};
  car_back_msg(car_run_data);
  int b=0;
  for(int r=0;r<run_time;r++)
  {
    b=r%5;
    if(b==0)
    {
      if(res>=0)
    {
      write(user_socket_fd,car_run_data,data_length);
    } 
     else
    {
      ROS_INFO("小车客户端未成功连接，无法发送后退信息");
    }
    }
    loop_rate.sleep();
  }
  car_stop_control();
}

void RobotCar::car_left_control(double length)
{
  ros::Time::init();
  length=abs(length);
  ros::Rate loop_rate(100);
  int run_time=length*100/95.7;
  uint8_t car_run_data[14]={0};
  car_left_msg(car_run_data);
  int b=0;
  for(int r=0;r<run_time;r++)
  {
    b=r%5;
    if(b==0)
    {
      if(res>=0)
    {
      write(user_socket_fd,car_run_data,data_length);
    } 
     else
    {
      ROS_INFO("小车客户端未成功连接，无法发送左移信息");
    }
    }
    loop_rate.sleep();
  }
  car_stop_control();
}

void RobotCar::car_right_control(double length)
{
  ros::Time::init();
  length=abs(length);
  ros::Rate loop_rate(100);
  int run_time=length*100/95.7;
  uint8_t car_run_data[14]={0};
  car_right_msg(car_run_data);
  int b=0;
  for(int r=0;r<run_time;r++)
  {
    b=r%5;
    if(b==0)
    {
      if(res>=0)
    {
      write(user_socket_fd,car_run_data,data_length);
    } 
     else
    {
      ROS_INFO("小车客户端未成功连接，无法发送右移信息");
    }
    }
    loop_rate.sleep();
  }
  car_stop_control();
}

void RobotCar::car_action(double dx,double dy)
{
  if(dx>0)
  {
    car_forward_control(dx);
  }
  else
  {
    car_back_control(dx);
  }
  if(dy>0)
  {
    car_left_control(dy);
  }
  else
  {
    car_right_control(dy);
  }
}
