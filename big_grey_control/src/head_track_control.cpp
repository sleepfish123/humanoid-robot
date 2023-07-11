#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include</home/yu/big_grey/src/big_grey_control/include/big_grey_control/RobotArm.h>
#include</home/yu/big_grey/src/big_grey_control/include/big_grey_control/Robot.h>
#include <sensor_msgs/JointState.h>
#include<iostream>

using namespace std;

float lf_degree=0,ud_degree=0;

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
  js.position[4]=0; // 右前后抬v
  js.position[5]=20; // 点头v-6.34
  js.position[6]=30; // 摇头v-15.75

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



void Motor_callback(std_msgs::Float32MultiArray head_position)
{
    // cout<<head_position.data[0]<<endl;
    // cout<<head_position.data[1]<<endl;
    //Head_Position.position.clear();
    // Head_Position.position.push_back(0.0);
    // Head_Position.position.push_back(0.0);
    // Head_Position.position.push_back(0.0);
    // Head_Position.position.push_back(0.0);
    // Head_Position.position.push_back(0.0);
    // Head_Position.position.push_back(head_position.data[1]);    //点头
    lf_degree = head_position.data[0];
    // Head_Position.position.push_back(30.00);    //摇头
    ud_degree = head_position.data[1];
    // Head_Position.position.push_back(0.0);
    // Head_Position.position.push_back(0.0);
    // Head_Position.position.push_back(0.0);
    // Head_Position.position.push_back(0.0);
    // Head_Position.position.push_back(0.0);
    // Head_Position.position.push_back(0.0);
    
    //

    // printf("%f  %f\n",head_position.data[1],head_position.data[0]);


}

int main(int argc, char  *argv[])
{
    
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"head_track_control");

    
    RobotArm Robot_Head(ARM_SERIAL_DEFAULT_NAME, ARM_SERIAL_DEFAULT_BAUDRATE);
    Robot_Head.arm_init();

    sensor_msgs::JointState Head_Position;   // 控制舵机的消息类型   string []  name           float  []  position
    Head_Position.position.clear();
    for(int i=0;i<2;i++)
     {
        Head_Position.position.push_back(0.0);
    }
    Head_Position.name.resize(2);
    // Head_Position.name.push_back("angle_YXB");
    // Head_Position.name.push_back("angle_YZB");
    // Head_Position.name.push_back("angle_YDB");
    // Head_Position.name.push_back("angle_YJQH");
    // Head_Position.name.push_back("angle_YJCT");
    // Head_Position.name.push_back("angle_ZXB");
    // Head_Position.name.push_back("angle_ZZB");
    // Head_Position.name.push_back("angle_ZZB");
    // Head_Position.name.push_back("angle_ZDB");
    // Head_Position.name.push_back("angle_ZJQH");
    // Head_Position.name.push_back("angle_ZSW");
    Head_Position.name.push_back("angle_DT");
    Head_Position.name.push_back("angle_YT");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("DJ_position",10,Motor_callback);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        // Robot_Head.arm_control(Head_Position);
        // printf("%f  %f\n",Head_Position.position[5],Head_Position.position[6]);
        // Head_Position.position[0] = 0.0;
        // Head_Position.position[1] = 0.0;
        // Head_Position.position[2] = 0.0;
        // Head_Position.position[3] = 0.0;
        // Head_Position.position[4] = 0.0;
        Head_Position.position[0] = ud_degree;
        Head_Position.position[1] = lf_degree;
        // Head_Position.position[0] = ud_degree;
        // Head_Position.position[1] = lf_degree;
        // Head_Position.position[5] = 20;
        // Head_Position.position[6] = 30;
        // Head_Position.position[7] = 0.0;
        // Head_Position.position[8] = 0.0;
        // Head_Position.position[9] = 0.0;
        // Head_Position.position[10] = 0.0;
        // Head_Position.position[11] = 0.0;
        // Head_Position.position[12] = 0.0;
        printf("%f %f\n",Head_Position.position[0] ,Head_Position.position[1] );


         Robot_Head.head_control(Head_Position);
        //arm_test();
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
