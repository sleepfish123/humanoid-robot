#include<big_grey_control/RobotClaw.h>
void claw_stop(uint8_t* claw_stop_array)
{
    claw_stop_array[0]=0x01;
    claw_stop_array[1]=0x06;
    claw_stop_array[2]=0x00;
    claw_stop_array[3]=0x02;
    claw_stop_array[4]=0x00;
    claw_stop_array[5]=0x01;
    claw_stop_array[6]=0xE9;
    claw_stop_array[7]=0xCA;
}

void claw_close(uint8_t* claw_close_array)//正转
{
    claw_close_array[0]=0x01;
    claw_close_array[1]=0x06;
    claw_close_array[2]=0x00;
    claw_close_array[3]=0x00;
    claw_close_array[4]=0x00;
    claw_close_array[5]=0x01;
    claw_close_array[6]=0x48;
    claw_close_array[7]=0x0A;
}
void claw_open(uint8_t* claw_open_array)//反转
{
    claw_open_array[0]=0x01;
    claw_open_array[1]=0x06;
    claw_open_array[2]=0x00;
    claw_open_array[3]=0x01;
    claw_open_array[4]=0x00;
    claw_open_array[5]=0x01;
    claw_open_array[6]=0x19;
    claw_open_array[7]=0xCA;

}

void claw_velocity(uint8_t* claw_velocity_array)
{
    claw_velocity_array[0]=0x01;
    claw_velocity_array[1]=0x06;
    claw_velocity_array[2]=0x00;
    claw_velocity_array[3]=0x05;
    claw_velocity_array[4]=0x0A;
    claw_velocity_array[5]=0x00;
    claw_velocity_array[6]=0x9F;
    claw_velocity_array[7]=0x6B;
}

void claw_extend(uint8_t* claw_extend_array)
{
    claw_extend_array[0]=0x01;
    claw_extend_array[1]=0x06;
    claw_extend_array[2]=0x00;
    claw_extend_array[3]=0x07;
    claw_extend_array[4]=0x01; // 0x02
    claw_extend_array[5]=0x11; // 0x00
    claw_extend_array[6]=0xF9;
    claw_extend_array[7]=0x97;
}

RobotClaw::RobotClaw(const std::string serial_name,int baudrate)
{
  setlocale(LC_ALL,"");
  ser_claw.setPort(serial_name);    
  ser_claw.setBaudrate(baudrate);
  if(ser_claw.isOpen())
  {
      ROS_INFO("***手爪串口已打开,无需重复打开***");
  }
  else
  {
    try
    {
        serial::Timeout to=serial::Timeout::simpleTimeout(1000);
        ser_claw.setTimeout(to);
        ser_claw.open();
    }
    catch(serial::IOException &e)
    {
        ROS_INFO("***不能打开手爪串口!***");
        sleep(10); // *********************
    }
  }
  if(ser_claw.isOpen())
  {
      ROS_INFO("***手爪串口已经打开***");
  }
  else
  {
      ROS_INFO("***手爪串口未打开***");
      sleep(10); // *********************
  }
}

RobotClaw::~RobotClaw()
{
  setlocale(LC_ALL,"");
  ROS_INFO("***手爪类析构函数被调用***");
  ser_claw.close();
}

  void RobotClaw::claw_close_msg(uint8_t *close)
  {
    close[0]=0x40;
    close[1]=0x31;
    close[2]=0x39;
    close[3]=0x39;
    close[4]=0x3a;
    close[5]=0x4d;
    close[6]=0x6f;
    close[7]=0x74;
    close[8]=0x6f;
    close[9]=0x72;
    close[10]=0x3a;
    close[11]=0x52;
    close[12]=0x75;
    close[13]=0x6e;
    close[14]=0x3a;
    close[15]=0x4d;
    close[16]=0x31;
    close[17]=0x20;
    close[18]=0x2d;
    close[19]=0x35;//0x31代表-100中1 的ASCLL码值，9为0x39
    close[20]=0x30;
    close[21]=0x30;
    close[22]=0x0d;
    close[23]=0x0a;
  }
  
  void RobotClaw::claw_open_msg(uint8_t *open)
  {
    open[0]=0x40;
    open[1]=0x31;
    open[2]=0x39;
    open[3]=0x39;
    open[4]=0x3a;
    open[5]=0x4d;
    open[6]=0x6f;
    open[7]=0x74;
    open[8]=0x6f;
    open[9]=0x72;
    open[10]=0x3a;
    open[11]=0x52;
    open[12]=0x75;
    open[13]=0x6e;
    open[14]=0x3a;
    open[15]=0x4d;
    open[16]=0x31;
    open[17]=0x20;
    //open[18]=0x2d;此位代表方向
    open[18]=0x35;//0x31代表-100中1 的ASCLL码值，9为0x39
    open[19]=0x30;
    open[20]=0x30;
    open[21]=0x0d;
    open[22]=0x0a;
  }

void RobotClaw::claw_open_control()
{
  uint8_t claw_velocity_array[8]={0};
  uint8_t claw_extend_array[8]={0};
  uint8_t claw_open_array[8]={0};
  uint8_t claw_stop_array[8]={0};
  size_t array_length=8;
  claw_velocity(claw_velocity_array);
  claw_extend(claw_extend_array);
  claw_close(claw_open_array);
  claw_stop(claw_stop_array);
  if(ser_claw.isOpen())
  {
    ser_claw.write(claw_velocity_array,array_length);
    usleep(TimeDelay);
    ser_claw.write(claw_extend_array,array_length);
    usleep(TimeDelay);
    ser_claw.write(claw_open_array,array_length);
    ROS_INFO("***已发送张开手爪信息***");
    // sleep(3);
    // ser_claw.write(claw_stop_array,array_length);
    // usleep(TimeDelay);
    // ROS_INFO("***已发送停止手爪指令***");
    claw_state=CLAW_OPEN;
  }
  else
  {
    ROS_INFO("***手爪串口未打开，无法发送张开手爪信息***");
    sleep(10); // *********************
  }
  
}

void RobotClaw::claw_close_control()
{
  uint8_t claw_velocity_array[8]={0};
  uint8_t claw_extend_array[8]={0};
  uint8_t claw_open_array[8]={0};
  uint8_t claw_stop_array[8]={0};
  size_t array_length=8;
  claw_velocity(claw_velocity_array);
  claw_extend(claw_extend_array);
  claw_open(claw_open_array);
  claw_stop(claw_stop_array);
  if(ser_claw.isOpen())
  {
    ser_claw.write(claw_velocity_array,array_length);
    usleep(TimeDelay);
    ser_claw.write(claw_extend_array,array_length);
    usleep(TimeDelay);
    ser_claw.write(claw_open_array,array_length);
    // sleep(1);
    ROS_INFO("***已发送闭合手爪信息***");
    // ser_claw.write(claw_stop_array,array_length);
    // usleep(TimeDelay);
    // ROS_INFO("***已发送停止手爪指令***");
    claw_state=CLAW_CLOSE;
  }
  else
  {
    ROS_INFO("***手爪串口未打开，无法发送闭合手爪信息***");
    sleep(10); // *********************
  }
  
}