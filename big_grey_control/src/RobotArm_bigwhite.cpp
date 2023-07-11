#include <big_grey_control/RobotArm.h>

uint8_t arm_control_YXB[12]={0};
uint8_t arm_control_YZB[12]={0};
uint8_t arm_control_YDB[12]={0};
uint8_t arm_control_YJCT[12]={0};
uint8_t arm_control_YJQH[12]={0};
uint8_t arm_control_DT[12]={0};
uint8_t arm_control_YT[12]={0};
uint8_t arm_control_ZXB[12]={0};
uint8_t arm_control_ZDB[12]={0};
uint8_t arm_control_ZZB[12]={0};
uint8_t arm_control_ZJCT[12]={0};
uint8_t arm_control_ZJQH[12]={0};
uint8_t arm_control_ZSW[12]={0};
uint8_t asynchronous_action[6]={0};

RobotArm::RobotArm(const char* serial_name,int baudrate)
{
    setlocale(LC_ALL,"");
    strcpy(this->serial_name,serial_name);
    ser_arm.setPort(this->serial_name);    //打开控制机械臂的串口
    ser_arm.setBaudrate(baudrate);
    if(ser_arm.isOpen())
    {
        ROS_INFO("***机械臂串口已打开,无需重复打开***");
    }
    else
    {
        try
        {
            serial::Timeout to=serial::Timeout::simpleTimeout(1000);
            ser_arm.setTimeout(to);
            ser_arm.open();
        }
        catch(serial::IOException &e)
        {
            ROS_INFO("***不能打开机械臂串口!***");
            sleep(10); // *********************
            exit(-1);
        }
    }
    if(ser_arm.isOpen())
    {
        ROS_INFO("***机械臂串口已经打开***");
    }
    else
    {
        ROS_INFO("***机械臂串口未打开***");
        sleep(10); // *********************
    }
    control_msg_length=13;
    action_msg_length=6;
    isInit=false;
    
    robot_arm_pub=nh.advertise<sensor_msgs::JointState>("/robot_arm_states",1000);
    robot_arm_states.name.resize(13);
    robot_arm_states.name.push_back("angle_YXB");
    robot_arm_states.name.push_back("angle_YZB");
    robot_arm_states.name.push_back("angle_YDB");
    robot_arm_states.name.push_back("angle_YJQH");
    robot_arm_states.name.push_back("angle_YJCT");
    robot_arm_states.name.push_back("angle_ZXB");
    robot_arm_states.name.push_back("angle_ZZB");
    robot_arm_states.name.push_back("angle_ZDB");
    robot_arm_states.name.push_back("angle_ZJQH");
    robot_arm_states.name.push_back("angle_ZJCT");
    robot_arm_states.name.push_back("angle_ZSW");
    robot_arm_states.name.push_back("angle_DT");
    robot_arm_states.name.push_back("angle_YT");
}

RobotArm::~RobotArm()
{
    setlocale(LC_ALL,"");
    ROS_INFO("***机械臂析构函数被调用***");
    cout<<"串口 "<<ser_arm.getPort()<<ser_arm.getBaudrate()<<" 已关闭"<<endl;
    ser_arm.close();
}

double RobotArm::angle2rad(double angle)
{
    return angle*M_PI/180.0;
}

double RobotArm::rad2angle(double rad)
{
    return rad*180.0/M_PI;
}

void RobotArm::arm_msg(uint8_t *arm_control_msgs,int id,double angle)
{
    int check_sum=0;
    uint8_t arm_control[13]={0};
   arm_control[0]=0xff;
   arm_control[1]=0xff;
   arm_control[2]=id;
   arm_control[3]=0x09;
   arm_control[4]=0x04;
   arm_control[5]=0x2a;

   switch (id)
   {
   case YDB_ID:
    angle_YDB=angle;
    arm_control[6]=(int)(-angle*YDB_SOLVE+YDB_INIT)&0XFF;
    arm_control[7]=(int)(-angle*YDB_SOLVE+YDB_INIT)>>8;
    arm_control[10]=0x64;
    arm_control[11]=0x00;
    break;
    case YZB_ID:
    angle_YZB=angle;
    arm_control[6]=(int)(angle*YZB_SOLVE+YZB_INIT)&0XFF;
    arm_control[7]=(int)(angle*YZB_SOLVE+YZB_INIT)>>8;
    arm_control[10]=0xC8;
    arm_control[11]=0x00;
    break;
    case YJQH_ID:
    angle_YJQH=angle;
    arm_control[6]=(int)(-angle*YJQH_SOLVE+YJQH_INIT)&0XFF;
    arm_control[7]=(int)(-angle*YJQH_SOLVE+YJQH_INIT)>>8;
    arm_control[10]=0x64;
    arm_control[11]=0x00;
    break;
    case YJCT_ID:
    angle_YJCT=angle;
    arm_control[6]=(int)(angle*YJCT_SOLVE+YJCT_INIT)&0XFF;
    arm_control[7]=(int)(angle*YJCT_SOLVE+YJCT_INIT)>>8;
    arm_control[10]=0x64;
    arm_control[11]=0x00;
    break;
    case YXB_ID:
    angle_YXB=angle;
    arm_control[6]=(int)(angle*YXB_SOLVE+YXB_INIT)&0XFF;
    arm_control[7]=(int)(angle*YXB_SOLVE+YXB_INIT)>>8;
    arm_control[10]=0x64;
    arm_control[11]=0x00;
    break;
    case ZSW_ID:
    angle_ZSW=angle;
    arm_control[7]=(int)(-angle*ZSW_SOLVE+ZSW_INIT)&0XFF;
    arm_control[6]=(int)(-angle*ZSW_SOLVE+ZSW_INIT)>>8;
    arm_control[10]=0x00;
    arm_control[11]=0x64;
    break;
    case ZXB_ID:
    angle_ZXB=angle;
    arm_control[6]=(int)(angle*ZXB_SOLVE+ZXB_INIT)&0XFF;
    arm_control[7]=(int)(angle*ZXB_SOLVE+ZXB_INIT)>>8;
    arm_control[10]=0x64;
    arm_control[11]=0x00;
    break;
    case ZZB_ID:
    angle_ZZB=angle;
    arm_control[6]=(int)(-angle*ZZB_SOLVE+ZZB_INIT)&0XFF;
    arm_control[7]=(int)(-angle*ZZB_SOLVE+ZZB_INIT)>>8;
    arm_control[10]=0x64;
    arm_control[11]=0x00;
    break;
    case ZDB_ID:
    angle_ZDB=angle;
    arm_control[6]=(int)(angle*ZDB_SOLVE+ZDB_INIT)&0XFF;
    arm_control[7]=(int)(angle*ZDB_SOLVE+ZDB_INIT)>>8;
    arm_control[10]=0x64;
    arm_control[11]=0x00;
    break;
    case ZJQH_ID:
    angle_ZJQH=angle;
    arm_control[6]=(int)(-angle*ZJQH_SOLVE+ZJQH_INIT)&0XFF;
    arm_control[7]=(int)(-angle*ZJQH_SOLVE+ZJQH_INIT)>>8;
    arm_control[10]=0x64;
    arm_control[11]=0x00;
    break;
    case ZJCT_ID:
    angle_ZJCT=angle;
    arm_control[6]=(int)(-angle*ZJCT_SOLVE+ZJCT_INIT)&0XFF;
    arm_control[7]=(int)(-angle*ZJCT_SOLVE+ZJCT_INIT)>>8;
    arm_control[10]=0x64;
    arm_control[11]=0x00;
    break;
    case DT_ID:
    angle_DT=angle;
    arm_control[7]=(int)(angle*DT_SOLVE+DT_INIT)&0XFF;
    arm_control[6]=(int)(angle*DT_SOLVE+DT_INIT)>>8;
    arm_control[10]=0x00;
    arm_control[11]=0x64;
    break;
    case YT_ID:
    angle_YT=angle;
    arm_control[7]=(int)(-angle*YT_SOLVE+YT_INIT)&0XFF;
    arm_control[6]=(int)(-angle*YT_SOLVE+YT_INIT)>>8;
    arm_control[10]=0x00;
    arm_control[11]=0x64;
    break;
   default:
       break;
   }
    arm_control[8]=0x00;
    arm_control[9]=0x00;
   for(int i=2;i<12;i++)
    {
        check_sum=check_sum+arm_control[i];
    }
    arm_control[12]=~(char(check_sum));
    memmove(arm_control_msgs,arm_control,sizeof(uint8_t)*13);
}

void RobotArm::action_command_msg(uint8_t *action)
{
    uint8_t action_tmp[6]={0};
   action_tmp[0]=0xff;
   action_tmp[1]=0xff;
   action_tmp[2]=0xfe;
   action_tmp[3]=0x02;
   action_tmp[4]=0x05;
   action_tmp[5]=0xfa;
   memmove(action,action_tmp,sizeof(uint8_t)*6);
}

void RobotArm::publish()
{
    robot_arm_states.position.clear();
    robot_arm_states.position.push_back(angle_YXB);
    robot_arm_states.position.push_back(angle_YZB);
    robot_arm_states.position.push_back(angle_YDB);
    robot_arm_states.position.push_back(angle_YJCT);
    robot_arm_states.position.push_back(angle_YJQH);
    robot_arm_states.position.push_back(angle_DT);
    robot_arm_states.position.push_back(angle_YT);
    robot_arm_states.position.push_back(angle_ZJQH);
    robot_arm_states.position.push_back(angle_ZJCT);
    robot_arm_states.position.push_back(angle_ZDB);
    robot_arm_states.position.push_back(angle_ZZB);
    robot_arm_states.position.push_back(angle_ZXB);
    robot_arm_states.position.push_back(angle_ZSW);
    robot_arm_pub.publish(robot_arm_states);
}

void RobotArm::arm_control(sensor_msgs::JointState &robot_arm_states)
{
    setlocale(LC_ALL,"");
    
    arm_msg(arm_control_YXB,YXB_ID,robot_arm_states.position[0]);
    arm_msg(arm_control_YZB,YZB_ID,robot_arm_states.position[1]);
    arm_msg(arm_control_YDB,YDB_ID,robot_arm_states.position[2]);
    arm_msg(arm_control_YJCT,YJCT_ID,robot_arm_states.position[3]);
    arm_msg(arm_control_YJQH,YJQH_ID,robot_arm_states.position[4]);
    arm_msg(arm_control_DT,DT_ID,robot_arm_states.position[5]);
    arm_msg(arm_control_YT,YT_ID,robot_arm_states.position[6]);
    arm_msg(arm_control_ZJQH,ZJQH_ID,robot_arm_states.position[7]);
    arm_msg(arm_control_ZJCT,ZJCT_ID,robot_arm_states.position[8]);
    arm_msg(arm_control_ZDB,ZDB_ID,robot_arm_states.position[9]);
    arm_msg(arm_control_ZZB,ZZB_ID,robot_arm_states.position[10]);
    arm_msg(arm_control_ZXB,ZXB_ID,robot_arm_states.position[11]);
    arm_msg(arm_control_ZSW,ZSW_ID,robot_arm_states.position[12]);
    action_command_msg(asynchronous_action);
    ser_arm.flush();
    if(ser_arm.isOpen())
    {
        for(int j=0;j<1;j++)
        {
        ser_arm.write(arm_control_DT,control_msg_length);
        usleep(100000);
        ser_arm.write(arm_control_YT,control_msg_length);
        usleep(100000);
        ser_arm.write(arm_control_ZJQH,control_msg_length);
        usleep(100000);
        ser_arm.write(arm_control_ZJCT,control_msg_length);
        usleep(100000);
        ser_arm.write(arm_control_ZDB,control_msg_length);
        usleep(100000);
        ser_arm.write(arm_control_ZZB,control_msg_length);
        usleep(100000);
        ser_arm.write(arm_control_ZXB,control_msg_length);
        usleep(100000);
        ser_arm.write(arm_control_YXB,control_msg_length);
        usleep(100000);
        ser_arm.write(arm_control_YZB,control_msg_length);
        usleep(100000);
        ser_arm.write(arm_control_YJCT,control_msg_length);
        usleep(100000);
        ser_arm.write(arm_control_YJQH,control_msg_length);
        usleep(100000);
        ser_arm.write(asynchronous_action,action_msg_length);
        usleep(100000);
        ser_arm.write(arm_control_YDB,control_msg_length);
        usleep(100000);
        ser_arm.write(asynchronous_action,action_msg_length);
        usleep(100000);
        ser_arm.write(arm_control_ZXB,control_msg_length);
        usleep(100000);
        ser_arm.write(asynchronous_action,action_msg_length);
        usleep(100000);
        ser_arm.write(arm_control_ZSW,control_msg_length);
        usleep(100000);
        ser_arm.write(asynchronous_action,action_msg_length);
        usleep(100000);
        }
        // ser_arm.flush();
        // ser_arm.write(arm_control_ZSW,control_msg_length);
        // usleep(100000);
        // ser_arm.write(asynchronous_action,action_msg_length);
        // usleep(100000);
    }
    else
    {
        ROS_INFO("***机械臂串口未打开，无法发送控制信息***");
        sleep(10); // *********************
    }
}

void RobotArm::head_control(sensor_msgs::JointState &robot_arm_states)
{
    setlocale(LC_ALL,"");
    
    // arm_msg(arm_control_YXB,YXB_ID,robot_arm_states.position[0]);
    // arm_msg(arm_control_YZB,YZB_ID,robot_arm_states.position[1]);
    // arm_msg(arm_control_YDB,YDB_ID,robot_arm_states.position[2]);
    // arm_msg(arm_control_YJCT,YJCT_ID,robot_arm_states.position[3]);
    // arm_msg(arm_control_YJQH,YJQH_ID,robot_arm_states.position[4]);
    arm_msg(arm_control_DT,DT_ID,robot_arm_states.position[0]);
    arm_msg(arm_control_YT,YT_ID,robot_arm_states.position[1]);
    // arm_msg(arm_control_ZJQH,ZJQH_ID,robot_arm_states.position[7]);
    // arm_msg(arm_control_ZJCT,ZJCT_ID,robot_arm_states.position[8]);
    // arm_msg(arm_control_ZDB,ZDB_ID,robot_arm_states.position[9]);
    // arm_msg(arm_control_ZZB,ZZB_ID,robot_arm_states.position[10]);
    // arm_msg(arm_control_ZXB,ZXB_ID,robot_arm_states.position[11]);
    // arm_msg(arm_control_ZSW,ZSW_ID,robot_arm_states.position[12]);
    action_command_msg(asynchronous_action);
    ser_arm.flush();
    if(ser_arm.isOpen())
    {
        for(int j=0;j<1;j++)
        {
        ser_arm.write(arm_control_DT,control_msg_length);
        usleep(50000);
        ser_arm.write(arm_control_YT,control_msg_length);
        usleep(50000);
        // ser_arm.write(arm_control_ZJQH,control_msg_length);
        // usleep(100000);
        // ser_arm.write(arm_control_ZJCT,control_msg_length);
        // usleep(100000);
        // ser_arm.write(arm_control_ZDB,control_msg_length);
        // usleep(100000);
        // ser_arm.write(arm_control_ZZB,control_msg_length);
        // usleep(100000);
        // ser_arm.write(arm_control_ZXB,control_msg_length);
        // usleep(100000);
        // ser_arm.write(arm_control_YXB,control_msg_length);
        // usleep(100000);
        // ser_arm.write(arm_control_YZB,control_msg_length);
        // usleep(100000);
        // ser_arm.write(arm_control_YJCT,control_msg_length);
        // usleep(100000);
        // ser_arm.write(arm_control_YJQH,control_msg_length);
        // usleep(100000);
        // ser_arm.write(asynchronous_action,action_msg_length);
        // usleep(100000);
        // ser_arm.write(arm_control_YDB,control_msg_length);
        // usleep(100000);
        // ser_arm.write(asynchronous_action,action_msg_length);
        // usleep(100000);
        // ser_arm.write(arm_control_ZXB,control_msg_length);
        // usleep(100000);
        // ser_arm.write(asynchronous_action,action_msg_length);
        // usleep(100000);
        // ser_arm.write(arm_control_ZSW,control_msg_length);
        // usleep(100000);
        ser_arm.write(asynchronous_action,action_msg_length);
        usleep(50000);
        }
        // ser_arm.flush();
        // ser_arm.write(arm_control_ZSW,control_msg_length);
        // usleep(100000);
        // ser_arm.write(asynchronous_action,action_msg_length);
        // usleep(100000);
    }
    else
    {
        ROS_INFO("***机械臂串口未打开，无法发送控制信息***");
        sleep(10); // *********************
    }
}

void RobotArm::arm_FBtrack_control(sensor_msgs::JointState &robot_arm_states)
{
    setlocale(LC_ALL,"");

    arm_msg(arm_control_YJQH,YJQH_ID,robot_arm_states.position[2]);
    arm_msg(arm_control_YZB,YZB_ID,robot_arm_states.position[0]-15);
    action_command_msg(asynchronous_action);
    ser_arm.flush();
    if(ser_arm.isOpen())
    {
        for(int j=0;j<1;j++)
        {
 
      
        ser_arm.write(arm_control_YJQH,control_msg_length);    //单独拿出来追踪深度，暂时注释掉
        usleep(10000);
        ser_arm.write(arm_control_YZB,control_msg_length);
        usleep(10000);
        ser_arm.write(asynchronous_action,action_msg_length);
        usleep(10000);
        }

    }
    else
    {
        ROS_INFO("***机械臂串口未打开，无法发送控制信息***");
        sleep(10); // *********************
    }
}


void RobotArm::arm_track_control(sensor_msgs::JointState &robot_arm_states)
{
    setlocale(LC_ALL,"");

    arm_msg(arm_control_YZB,YZB_ID,robot_arm_states.position[0]);
    arm_msg(arm_control_YDB,YDB_ID,robot_arm_states.position[1]);
    arm_msg(arm_control_YJQH,YJQH_ID,robot_arm_states.position[2]);

    action_command_msg(asynchronous_action);
    ser_arm.flush();
    if(ser_arm.isOpen())
    {
        for(int j=0;j<1;j++)
        {
 
        ser_arm.write(arm_control_YDB,control_msg_length);
        usleep(10000);
        ser_arm.write(arm_control_YZB,control_msg_length);
        usleep(10000);
        // ser_arm.write(arm_control_YJQH,control_msg_length);    //单独拿出来追踪深度，暂时注释掉
        // usleep(10000);
        ser_arm.write(asynchronous_action,action_msg_length);
        usleep(10000);
        }

    }
    else
    {
        ROS_INFO("***机械臂串口未打开，无法发送控制信息***");
        sleep(10); // *********************
    }
}


void RobotArm::arm_init()
{
    setlocale(LC_ALL,"");
    arm_msg(arm_control_YXB,YXB_ID,30.0);
    arm_msg(arm_control_YZB,YZB_ID,70.0);
    arm_msg(arm_control_YDB,YDB_ID,0.0);
    arm_msg(arm_control_YJCT,YJCT_ID,0.0);
    arm_msg(arm_control_YJQH,YJQH_ID,0.0);
    arm_msg(arm_control_DT,DT_ID,0.0);
    arm_msg(arm_control_YT,YT_ID,0.0);
    arm_msg(arm_control_ZJQH,ZJQH_ID,0.0);
    arm_msg(arm_control_ZJCT,ZJCT_ID,0.0);
    arm_msg(arm_control_ZDB,ZDB_ID,0.0);
    arm_msg(arm_control_ZZB,ZZB_ID,0.0);
    arm_msg(arm_control_ZXB,ZXB_ID,0.0);
    arm_msg(arm_control_ZSW,ZSW_ID,0.0);
    action_command_msg(asynchronous_action);
    if(ser_arm.isOpen())
    {
        for(int j=0;j<1;j++)
        {
            cout << "*****************fasong" << endl;
            ser_arm.write(arm_control_YXB,control_msg_length);
            usleep(100000);
            ser_arm.write(arm_control_YZB,control_msg_length);
            usleep(100000);
            ser_arm.write(arm_control_YDB,control_msg_length);
            usleep(100000);
            ser_arm.write(arm_control_YJCT,control_msg_length);
            usleep(100000);
            ser_arm.write(arm_control_YJQH,control_msg_length);
            usleep(100000);
            ser_arm.write(arm_control_DT,control_msg_length);
            usleep(100000);
            ser_arm.write(arm_control_YT,control_msg_length);
            usleep(100000);
            ser_arm.write(arm_control_ZJQH,control_msg_length);
            usleep(100000);
            ser_arm.write(arm_control_ZJCT,control_msg_length);
            usleep(100000);
            ser_arm.write(arm_control_ZDB,control_msg_length);
            usleep(100000);
            ser_arm.write(arm_control_ZZB,control_msg_length);
            usleep(100000);
            ser_arm.write(arm_control_ZXB,control_msg_length);
            usleep(100000);
            ser_arm.write(arm_control_ZSW,control_msg_length);
            usleep(100000);
            ser_arm.write(asynchronous_action,action_msg_length);
            usleep(100000);
        }
        isInit=true;
    }
    else
    {
        ROS_INFO("***机械臂串口未打开，无法发送初始化控制信息***");
        sleep(10); // *********************
        isInit=false;
    }

}

void RobotArm::joint_state_assign(sensor_msgs::JointState &joint_state)
{
    angle_YXB= robot_arm_states.position[0];
    angle_YZB=robot_arm_states.position[1];
    angle_YDB=robot_arm_states.position[2];
    angle_YJCT=robot_arm_states.position[3];
    angle_YJQH=robot_arm_states.position[4];
    angle_DT=robot_arm_states.position[6];
    angle_YT=robot_arm_states.position[7];
    angle_ZJQH=robot_arm_states.position[8];
    angle_ZJCT=robot_arm_states.position[9];
    angle_ZDB=robot_arm_states.position[10];
    angle_ZZB=robot_arm_states.position[11];
    angle_ZXB=robot_arm_states.position[12];
    angle_ZSW=robot_arm_states.position[13];
}

// ostream & operator<<(ostream &cout,sensor_msgs::JointState &joint_state)
// {
//     if(joint_state.position.empty()||joint_state.name.empty())
//     {
//         cout<<"***joint_state position or name vector is empty***";
//         return cout;
//     }
//     for(int i=0;i<joint_state.position.size();i++)
//     {
//         cout<<"joint name: "<<joint_state.name.at(i)<<" position: "<<joint_state.position[i]<<endl;
//         // <<" velocity: "<<joint_state.velocity[i]<<" effort: "<<joint_state.effort[i];
//     }
//     return cout;
// }
