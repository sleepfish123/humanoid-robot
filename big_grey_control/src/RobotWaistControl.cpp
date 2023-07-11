/*
* 文件名：RobotWaistControl.cpp
* 描述：大灰腰部控制代码
* 修改人：于川
* 修改时间：2022-09-08
* 修改内容：新增
*/

#include<big_grey_control/RobotWaist.h>

uint8_t waist_sensor_recv_data[21]={0};
uint8_t foot_sensor_recv_data[37]={0};
RobotWaist::RobotWaist(int socket_fd,const std::string waist_sensor_serial_name,int waist_sensor_baudrate,
                          const std::string foot_sensor_serial_name,int foot_sensor_baudrate)
{
  setlocale(LC_ALL,"");
  user_socket_fd=socket_fd;
  addr.sin_family=AF_INET;
  addr.sin_port=htons(Port);
  addr.sin_addr.s_addr=inet_addr(ServerIP);
  res=connect(socket_fd,(struct sockaddr*)&addr,sizeof(addr));
  if(res==-1)
  {
    ROS_INFO("***连接失败***");
    exit(-1);
  }
  else
  {
    ROS_INFO("***连接成功***");
  }
  length=14;
  ser_waist_sensor.setPort(waist_sensor_serial_name);    
  ser_waist_sensor.setBaudrate(waist_sensor_baudrate);
  if(ser_waist_sensor.isOpen())
  {
      ROS_INFO("***腰部传感器串口已打开,无需重复打开***");
  }
  else
  {
    try
    {
        serial::Timeout to=serial::Timeout::simpleTimeout(1000);
        ser_waist_sensor.setTimeout(to);
        ser_waist_sensor.open();
    }
    catch(serial::IOException &e)
    {
        ROS_INFO("***不能打开腰部传感器串口!***");
    }
  }
  if(ser_waist_sensor.isOpen())
  {
      ROS_INFO("***腰部传感器串口已经打开***");
  }
  else
  {
      ROS_INFO("***腰部传感器串口未打开***");
  }

  ser_foot_sensor.setPort(foot_sensor_serial_name);    
  ser_foot_sensor.setBaudrate(foot_sensor_baudrate);
  if(ser_foot_sensor.isOpen())
  {
      ROS_INFO("***足部传感器串口已打开,无需重复打开***");
  }
  else
  {
    try
    {
        serial::Timeout to=serial::Timeout::simpleTimeout(1000);
        ser_foot_sensor.setTimeout(to);
        ser_foot_sensor.open();
    }
    catch(serial::IOException &e)
    {
        ROS_INFO("***不能打开足部串口!***");
    }
  }
  if(ser_foot_sensor.isOpen())
  {
      ROS_INFO("***足部串口已经打开***");
  }
  else
  {
      ROS_INFO("***足部串口未打开***");
  }
}

RobotWaist::RobotWaist(int socket_fd,const std::string waist_sensor_serial_name,int waist_sensor_baudrate)
{
  setlocale(LC_ALL,"");
  user_socket_fd=socket_fd;
  addr.sin_family=AF_INET;
  addr.sin_port=htons(Port);
  addr.sin_addr.s_addr=inet_addr(ServerIP);
  res=connect(socket_fd,(struct sockaddr*)&addr,sizeof(addr));
  if(res==-1)
  {
    ROS_INFO("***连接失败***");
    exit(-1);
  }
  else
  {
    ROS_INFO("***连接成功***");
  }
  length=14;
  ser_waist_sensor.setPort(waist_sensor_serial_name);    
  ser_waist_sensor.setBaudrate(waist_sensor_baudrate);
  if(ser_waist_sensor.isOpen())
  {
      ROS_INFO("***腰部传感器串口已打开,无需重复打开***");
  }
  else
  {
    try
    {
        serial::Timeout to=serial::Timeout::simpleTimeout(1000);
        ser_waist_sensor.setTimeout(to);
        ser_waist_sensor.open();
    }
    catch(serial::IOException &e)
    {
        ROS_INFO("***不能打开腰部传感器串口!***");
    }
  }
  if(ser_waist_sensor.isOpen())
  {
      ROS_INFO("***腰部传感器串口已经打开***");
  }
  else
  {
      ROS_INFO("***腰部传感器串口未打开***");
  }
}

RobotWaist::RobotWaist(int socket_fd,int foot_sensor_baudrate,const std::string foot_sensor_serial_name)
{
  setlocale(LC_ALL,"");
  user_socket_fd=socket_fd;
  addr.sin_family=AF_INET;
  addr.sin_port=htons(Port);
  addr.sin_addr.s_addr=inet_addr(ServerIP);
  res=connect(socket_fd,(struct sockaddr*)&addr,sizeof(addr));
  if(res==-1)
  {
    ROS_INFO("***连接失败***");
    exit(-1);
  }
  else
  {
    ROS_INFO("***连接成功***");
  }
  length=14;
  ser_foot_sensor.setPort(foot_sensor_serial_name);    
  ser_foot_sensor.setBaudrate(foot_sensor_baudrate);
  if(ser_foot_sensor.isOpen())
  {
      ROS_INFO("***足部传感器串口已打开,无需重复打开***");
  }
  else
  {
    try
    {
        serial::Timeout to=serial::Timeout::simpleTimeout(1000);
        ser_foot_sensor.setTimeout(to);
        ser_foot_sensor.open();
    }
    catch(serial::IOException &e)
    {
        ROS_INFO("***不能打开足部串口!***");
    }
  }
  if(ser_foot_sensor.isOpen())
  {
      ROS_INFO("***足部串口已经打开***");
  }
  else
  {
      ROS_INFO("***足部串口未打开***");
  }
}
void RobotWaist::foot_sensor_clear()
{
  uint8_t send_data[8]={0x01,0x06,0x00,0x43,0x00,0x0a,0xf8,0x19};
  if(ser_foot_sensor.isOpen())
  {
    ser_foot_sensor.write(send_data,8);
  }
  else
  {
    ROS_INFO("客户端并未创建成功，无法发送腰部控制信息");
  }
  

}
RobotWaist::RobotWaist(int socket_fd)
{
  setlocale(LC_ALL,"");
  user_socket_fd=socket_fd;
  addr.sin_family=AF_INET;
  addr.sin_port=htons(Port);
  addr.sin_addr.s_addr=inet_addr(ServerIP);
  res=connect(socket_fd,(struct sockaddr*)&addr,sizeof(addr));
  if(res==-1)
  {
    ROS_INFO("***连接失败***");
    exit(-1);
  }
  else
  {
    ROS_INFO("***连接成功***");
  }
  length=14;
}

RobotWaist::~RobotWaist()
{
  setlocale(LC_ALL,"");
  ROS_INFO("***腰部类析构函数被调用***");
  close(user_socket_fd);
}

void RobotWaist::read_waist_sensor(double *Gan)
{
  int recv_data[6]={0};
  uint8_t send_data[8]={0x01,0x04,0x00,0x00,0x00,0x08,0xf1,0xcc};
  if(ser_waist_sensor.isOpen())
  {
    ser_waist_sensor.write(send_data,8);
    ser_waist_sensor.read(waist_sensor_recv_data,21);
    for(int i=0;i<6;i++)
    {
      recv_data[i]=waist_sensor_recv_data[i+5];
    }
    Gan[0]=(4080.0-(recv_data[2]*256.0+recv_data[3]))/12.64;//杆一
    Gan[1]=(4080.0-(recv_data[4]*256.0+recv_data[5]))/12.64;//杆二
    Gan[2]=(4080.0-(recv_data[0]*256.0+recv_data[1]))/12.64;//杆三
  }
  else
  {
    ROS_INFO("串口未打开，无法读取腰部控制信息");
  }
}

double RobotWaist::read_foot_sensor()
{
  int recv_data[3]={0};
  uint8_t send_data[8]={0x01,0x03,0x00,0x10,0x00,0x10,0x45,0xc3};
  if(ser_foot_sensor.isOpen())
  {
    ser_foot_sensor.write(send_data,8);
    usleep(100*1000);
    ser_foot_sensor.read(foot_sensor_recv_data,37);
    if(foot_sensor_recv_data[0]==0x01&&foot_sensor_recv_data[1]==0x03&&foot_sensor_recv_data[2]==0x20)
    {
      for(int i=0;i<3;i++)
      {
        recv_data[i]=foot_sensor_recv_data[i+3];
        #ifdef DEBUGOUTPUT
        ROS_INFO_STREAM (recv_data[i]);
        #endif
      }
      if(recv_data[2]==0)
      {
        return (recv_data[0]*256.0+recv_data[1])/53.25;
      }
      else 
      {
        #ifdef DEBUGOUTPUT
        ROS_INFO_STREAM("************"<<~recv_data[0]<<"  "<<~recv_data[1]<<"  "
        <<-((~recv_data[0])*256.0+(~recv_data[1]))/53.25);
        #endif
        uint8_t recv_data_0,recv_data_1;
        recv_data_0=~recv_data[0];
        recv_data_1=~recv_data[1];
        return  -((recv_data_0)*256.0+(recv_data_1))/53.25;
      }
    }
    else
    {
      memset(foot_sensor_recv_data,0,37);
    }
  }
  else
  {
    ROS_INFO("足部传感器串口未打开，无法发送读取指令");
    return FOOT_SENSOR_ERROR;
  }
}

void RobotWaist::waist_up_msg(uint8_t *up,int8_t v1,int8_t v2,int8_t v3)
{
  up[0]=0xfb; //数据头1
  up[1]=0x75; //数据头2
  up[2]=0x00; //编号位
  up[3]=0x42; //模式为速度模式
  up[4]=0x00; //脚部电机高位
  up[5]=0x00;//脚步电机低位
  if(v1<0)
  {
  up[6]=0xff;               //杆1
  up[7]=256+v1; 
  }
  else 
  {
  up[6]=0x00;               //杆1
  up[7]=v1; 
  }   
if(v2<0)
{
up[8]=0xff;               //杆2
up[9]=256+v2; 
}
else 
{
up[8]=0x00;               //杆2
up[9]=v2; 
}   
if(v3<0)
{
up[10]=0xff;               //杆3
up[11]=256+v1; 
}
else 
{
up[10]=0x00;               //杆3
up[11]=v1; 
}   
up[12]=0xef;
up[13]=0x5c;
}

void RobotWaist::stop_msg(uint8_t *stop)
{
  stop[0]=0xfb;
  stop[1]=0x75;                   
  stop[2]=0x00;                   //编号位
  stop[3]=0x42;               //模式位速度模式
  stop[4]=7000>>8;
  stop[5]=7000&0xff;
  stop[6]=7000>>8;
  stop[7]=7000&0xff;
  stop[8]=7000>>8;                  
  stop[9]=7000&0xff;
  stop[10]=7000>>8;
  stop[11]=7000&0xff;
  stop[12]=0xef;
  stop[13]=0x5c;
}

void RobotWaist::foot_msg(uint8_t *foot,int8_t s,uint8_t velocity)
{
  foot[0]=0xfb;
  foot[1]=0x75;
  foot[2]=0x00;                
  foot[3]=0x42;           
  foot[4]=0x00;//正转
  foot[5]=velocity;
  if(s==FOOT_REVERSE_ROTATION)
  {
    foot[4]=0xff;//反转
    // uint8_t temp=~velocity;
    // foot[5]=temp;
    foot[5]=velocity;
  }
  foot[6]=0x00;
  foot[7]=0x00;
  foot[8]=0x00;                  
  foot[9]=0x00;
  foot[10]=0x00;
  foot[11]=0x00;
  foot[12]=0xef;
  foot[13]=0x5c;
}

void RobotWaist::foot_action_control(double foot_angle)
{
  setlocale(LC_ALL,"");
  if(foot_angle<-90)
  {
    ROS_INFO("足部角度过小，已转化为最小限制角度");
    foot_angle=-90;
  }
  else if(foot_angle>90)
  {
    ROS_INFO("足部角度过大，已转化为最大限制角度");
    foot_angle=90;
  }
  foot_angle=-foot_angle;
  double current_degree=0;
  int rotation_flag;
  while(1)
  {
    current_degree=read_foot_sensor();
    if(current_degree==FOOT_SENSOR_ERROR)
    {
      ROS_INFO("足部编码器无法获取信息，控制受限");
      break;
    }
    if(foot_angle-current_degree<0)
    {
      rotation_flag=FOOT_FORWARD_ROTATION;
    }
    else
    {
      rotation_flag=FOOT_REVERSE_ROTATION;
    }
    #ifdef DEBUGOUTPUT
    ROS_INFO_STREAM("角度值: "<<current_degree);
    #endif
    if(abs(current_degree-foot_angle)<0.05) 
    {
      stop_control();
      ROS_INFO_STREAM("足部已经运动至指定位置");
      break;
    }
    else{
    if(abs(current_degree-foot_angle)>3&&rotation_flag==FOOT_FORWARD_ROTATION) foot_msg(foot_data,rotation_flag,0x60);
    else if(1<=abs(current_degree-foot_angle)<=3&&rotation_flag==FOOT_FORWARD_ROTATION) foot_msg(foot_data,rotation_flag,0x0d);
    else if(abs(current_degree-foot_angle)>3&&rotation_flag==FOOT_REVERSE_ROTATION) foot_msg(foot_data,rotation_flag,0xb0);
    else if(1<=abs(current_degree-foot_angle)<=3&&rotation_flag==FOOT_REVERSE_ROTATION) foot_msg(foot_data,rotation_flag,0xf2);
    if(res>=0)
    {
      write(user_socket_fd,foot_data,length);
    }
    else
    {
      ROS_INFO("客户端并未创建成功，无法发送腰部控制信息");
    }}
  }
}

void RobotWaist::waist_time_msg(uint8_t *data,int8_t l1_action, int8_t l2_action,int8_t l3_action)
{
  data[0]=0xfb;
  data[1]=0x75;                   //数据头
  data[2]=0x00;                   //编号位
  data[3]=0x42;               //模式位速度模式
  data[4]=0x00;                //脚部电机高位
  data[5]=0x00;                //脚部电机低位
  data[6]=0x00;               
  data[7]=0x00;  
  data[8]=0x00;              
  data[9]=0x00; 
  data[10]=0x00;               
  data[11]=0x00; 
  switch (l1_action)
  {
  case WAIST_DOWN:
    data[6]=0xff;               
    data[7]=206;  
    break;
  case WAIST_UP:
    data[6]=0x00;               
    data[7]=50;   
    break;
  case WAIST_STOP:
    data[6]=7000>>8;               
    data[7]=7000&0xff;        
    break;
  }
  switch (l2_action)
  {
  case WAIST_DOWN:
    data[8]=0xff;               
    data[9]=206;  
    break;
  case WAIST_UP:
    data[8]=0x00;               
    data[9]=50;   
    break;
  case WAIST_STOP:
    data[8]=7000>>8;               
    data[9]=7000&0xff;        
    break;
  }
  switch (l3_action)
  {
  case WAIST_DOWN:
    data[10]=0xff;               
    data[11]=206;  
    break;
  case WAIST_UP:
    data[10]=0x00;               
    data[11]=50;   
    break;
  case WAIST_STOP:
    data[10]=7000>>8;               
    data[11]=7000&0xff;        
    break;
  }
  data[12]=0xef;
  data[13]=0x5c;
}

void RobotWaist::waist_down_msg(uint8_t *down)
{
  down[0]=0xfb;
  down[1]=0x75;                   //数据头
  down[2]=0x00;                   //编号位
  down[3]=0x42;               //模式位速度模式
  down[4]=0x00;
  down[5]=0x00;
  down[6]=0xff;
  down[7]=0xec;
  down[8]=0xff;                  
  down[9]=0xec;
  down[10]=0xff;
  down[11]=0xec;
  down[12]=0xef;
  down[13]=0x5c;
}

void RobotWaist::waist_action_control(double l1,double l2,double l3)
{
  setlocale(LC_ALL,"");
  ros::Time::init();
  ros::Rate loop(5.0);
  int8_t l1_action=0;
  int8_t l2_action=0;
  int8_t l3_action=0;    
  if(l1<0)
  {
    l1_action=WAIST_DOWN;
  }
  else
  {
    l1_action=WAIST_UP;
  }
  if(l2<0)
  {
    l2_action=WAIST_DOWN;
  }
  else
  {
      l2_action=WAIST_UP;
  }
  if(l3<0)
  {
    l3_action=WAIST_DOWN;
  }
  else
  {
      l3_action=WAIST_UP;
  }
  waist_time_msg(waist_data, l1_action,l2_action, l3_action);
  int run_time1=(abs(l1)/3.1535*100)/4.95;
  int run_time2=(abs(l2)/3.1535*100)/4.95;
  int run_time3=(abs(l3)/3.1535*100)/4.95;

  if(res>=0)
  {
    write(user_socket_fd,waist_data,length);
  }
  else
  {
    ROS_INFO("客户端并未创建成功，无法发送腰部控制信息");
  }
  for (size_t i = 0; i < 4000; i++)
  {
    if(i==run_time1||i==run_time2||i==run_time3)
    {
      if(run_time1==i)
      {
        l1_action=WAIST_STOP;
      }
      if(run_time2==i)
      {
        l2_action=WAIST_STOP;
      }
      if(run_time3==i)
      {
        l3_action=WAIST_STOP;
      }
    }
    waist_time_msg(waist_data, l1_action,l2_action, l3_action);
    if(res>=0)
    {
      write(user_socket_fd,waist_data,length);
      loop.sleep();
    }
    else
    {
      ROS_INFO("客户端并未创建成功，无法发送腰部控制信息");
    }
    if(l1_action==WAIST_STOP&&l2_action==WAIST_STOP&&l3_action==WAIST_STOP)
    {
      ROS_INFO("腰部已移动到指定位置");
      break;
    }
  }
  
}

void RobotWaist::stop_control()
{
  setlocale(LC_ALL,"");
  ros::Time::init();
  ros::Rate loop(5.0);
  stop_msg(stop_data);
  if(res>=0)
  {
  write(user_socket_fd,stop_data,length);
  loop.sleep();
  write(user_socket_fd,stop_data,length);
  loop.sleep();
  write(user_socket_fd,stop_data,length);
  loop.sleep();
  }
  else
  {
    ROS_INFO("客户端并未创建成功，无法发送腰部与足部电机停止指令");
  }
}

void RobotWaist::foot_waist_action_control(double swing,double bend,double updown,double foot_angle)
{
  double hz=updown;
  double theta6=swing;
  double theta7=bend;
  double R=130;
  double l=641.94;  //%动平台的半径和套筒高度
  double  c6=cos(theta6*M_PI/180.0);
  double	 s6=sin(theta6*M_PI/180.0);
  double   c7=cos(theta7*M_PI/180.0);
  double	 s7=sin(theta7*M_PI/180.0); 
  double   d5=hz;  //%单位是mm
  double  z12= pow(3,0.5);
  double L1=-l+sqrt(pow(0.5*R-0.5*R*c7-z12*0.5*R*s6*s7,2)+pow (z12*0.5*R-z12*0.5*R*c6,2)+pow(-0.5*R*s7+z12*0.5*R*s6*c7+l+d5,2) ) ;
  double L2=-l+sqrt(pow(0.5*R-0.5*R*c7+z12*0.5*R*s6*s7,2)+pow (-1*z12*0.5*R+z12*0.5*R*c6,2)+pow(-0.5*R*s7-z12*0.5*R*s6*c7+l+d5,2));
  double L3=-l+sqrt(pow(-R+R*c7,2)+pow(R*s7+l+d5,2));
  cout<<"l1: "<<L1<<"mm l2:"<<L2<<"mm  l3: "<<L3<<"mm"<<endl;
  foot_action_control(foot_angle);
  waist_action_control(L1/10.0,L2/10.0,L3/10.0);
  
}
