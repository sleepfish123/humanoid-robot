#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<big_grey_control/RobotArm.h>
#include<big_grey_control/Robot.h>
#include <sensor_msgs/JointState.h>
#include<iostream>

using namespace std;

float lf_degree=0,ud_degree = 0,fb_degree = 0;
float last_fb_degree = 0;
float next_fb_degree;
float offset_dead_block = 278;// 设置深度追踪的偏移量死区

void arm_test()
{
  RobotArm RobotArm_(ARM_SERIAL_DEFAULT_NAME, ARM_SERIAL_DEFAULT_BAUDRATE);
  RobotArm_.arm_init();
  int flag;
  cout << "是否让初始化结束，开始动作";
  cin >> flag;
  // sleep(2);
  sensor_msgs::JointState js;
  js.position.clear();
  for(int i=0;i<13;i++)
  {
    js.position.push_back(0.0);
  }
 

    /******右的*******/
  js.position[0]=0; // 右小臂
  js.position[1]=0; // 右肘部v
  js.position[2]=0.; // 右大臂v
  js.position[3]=0; // 右侧抬，向外抬给负值v
  js.position[4]=25; // 右前后抬v
  js.position[5]=0; // 点头v-6.34
  js.position[6]=0; // 摇头v-15.75

  /******左的*******/
  js.position[7]=0.0;
  js.position[8]=0.0; //向外抬给负值
  js.position[9]=0.0;
  js.position[10]=0.0;
  js.position[11]=0.0;
  js.position[12]=0.0;
  
  RobotArm_.arm_control(js);
  cout << "动作结束，是否初始化";
  cin >> flag;
  RobotArm_.arm_init();
}

//定义 前后移动      舵机控制函数      右肩前后
float fb_servo(float offset_z)
{
    float fb_kp=0.001;     //待定
    float delta_degree;

    //设置调节死区
    if(abs(offset_z)<offset_dead_block)
     {
        offset_z = 0;
     }
    delta_degree = fb_kp * offset_z;
    next_fb_degree = last_fb_degree + delta_degree;
    

    //限幅
    if(next_fb_degree < -30)
        next_fb_degree = -30;                                         //幅度待定
    else if (next_fb_degree > 66)
        next_fb_degree = 66;

    last_fb_degree = next_fb_degree;
     return next_fb_degree;   
  
}

void Motor_callback(std_msgs::Float32MultiArray head_position)
{
    
    lf_degree = head_position.data[0];
    ud_degree = head_position.data[1];
    //ROS_INFO("Motor_callback回调函数已调用!");
    //fb_degree = head_position.data[2];
    // printf("%f  %f\n",head_position.data[1],head_position.data[0]);
}

void Depth_track_callback(std_msgs::Float32MultiArray goal_uvd)
{
    float depth;
    depth = goal_uvd.data[2];
    next_fb_degree = fb_servo(depth);
    //ROS_INFO("Depth_track_callback回调函数已调用!");
    //printf("%f\n",next_fb_degree);
}


int main(int argc, char  *argv[])
{
    int  count0=0,flag;  //继续进行的标志位   
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"arm_track_control");

    
    RobotArm Robot_trackarm(ARM_SERIAL_DEFAULT_NAME, ARM_SERIAL_DEFAULT_BAUDRATE);
    Robot_trackarm.arm_init();

    sensor_msgs::JointState trackarm_Position;   // 控制舵机的消息类型   string []  name           float  []  position
    trackarm_Position.position.clear();
    for(int i=0;i<3;i++)
     {
        trackarm_Position.position.push_back(0.0);
    }
    trackarm_Position.name.resize(3);
    trackarm_Position.name.push_back("angle_YZB");     //肘部 上下
    trackarm_Position.name.push_back("angle_YDB");    //大臂 左右
    trackarm_Position.name.push_back("angle_YJQH");  //大臂  前后

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("DJ_position",100,Motor_callback);
    

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        count0++;//计时器的作用
        trackarm_Position.position[0] = ud_degree;
        trackarm_Position.position[1] = lf_degree;
        //trackarm_Position.position[2] = fb_degree;
        //printf("%f %f %f\n",trackarm_Position.position[0] ,trackarm_Position.position[1],trackarm_Position.position[2]  );

        Robot_trackarm.arm_track_control(trackarm_Position);
         //arm_test();


        ros::spinOnce();
        loop_rate.sleep();

        if(count0==2000)   //暂时认为5秒内，目标中心已追踪到达图像中心 , 可根据调试情况酌情修改
        {
          cout<<"二维追踪结束，是否继续？"<<endl;
          cin>>flag;
          break;
        }
    }

    cout<<"开始深度追踪"<<endl;
    ros::Subscriber sub1 = nh.subscribe("goal_center_pixel_coordinate",100,Depth_track_callback);

    ros::Rate loop_rate1(100);
    while (ros::ok())
    {
      
      trackarm_Position.position[2] = next_fb_degree ;
      printf("%f %f\n",next_fb_degree,trackarm_Position.position[2]);
      Robot_trackarm.arm_FBtrack_control(trackarm_Position);
      //usleep(500000);

      ros::spinOnce();
      loop_rate1.sleep();

      //下边继续写深度控制舵机

    }
    return 0;
}
