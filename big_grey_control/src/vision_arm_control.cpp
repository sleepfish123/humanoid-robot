#include<ros/ros.h>
#include<iostream>
#include<std_msgs/Float32MultiArray.h>
using namespace std;



float offset_dead_block = 30;// 设置偏移量死区
float next_lf_degree,next_ud_degree=90,next_fb_degree;
float last_lf_degree=0,last_ud_degree=90,last_fb_degree=0;
float u0 = 320,v0 = 240;  //假设图像中心像素坐标为(0,0);      image_depth_rgb.cpp中查阅  图像中心点像素坐标为（320，240）
//如果摄像头安装在小臂上方，则需要将目标保持在图像中心稍微靠下方的地方，则期望的v0 可能需要往小微调  （320 ，<240)

//定义 左右移动      舵机控制函数      大臂
float lf_servo(float offset_x)
{
    float lf_kp=0.008;
    float delta_degree;

    //设置调节死区
    if(abs(offset_x)<offset_dead_block)
     {
        offset_x = 0;
     }
    delta_degree = lf_kp * offset_x;
    next_lf_degree = last_lf_degree + delta_degree;
    

    //限幅
    if(next_lf_degree < -10)
        next_lf_degree = -10;
    else if (next_lf_degree > 90)
        next_lf_degree = 90;

    last_lf_degree = next_lf_degree;
     return next_lf_degree;   
  
}

//定义 上下移动    舵机控制函数     肘部
float ud_servo(float offset_y)
{
    float ud_kp = -0.003;
    float delta_degree;

    //设置调节死区
    if(abs(offset_y)<offset_dead_block)
        offset_y = 0;

    delta_degree = ud_kp * offset_y; 
    next_ud_degree = last_ud_degree + delta_degree;
    

    //限幅
    if(next_ud_degree < 0)
        next_ud_degree = 0;
    else if (next_ud_degree > 90)
        next_ud_degree = 90;

    last_ud_degree = next_ud_degree;
     return next_ud_degree;   
}

//定义 前后移动      舵机控制函数      右肩前后
float fb_servo(float offset_z)
{
    float fb_kp=0.008;     //待定
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

void offset_callback(std_msgs::Float32MultiArray goal_uvd)
{
    
    float offset[3]={0,0,0};

    offset[0] = goal_uvd.data[0] - u0;
    offset[1] = goal_uvd.data[1] - v0;
    offset[2] = goal_uvd.data[2];

    next_lf_degree = lf_servo(offset[0]);
    next_ud_degree = ud_servo(offset[1]);
    //next_fb_degree = fb_servo(offset[2]);  //深度追踪等二维追踪结束后再进行，暂时放到arm_track_control.cpp中。
    // ROS_INFO("回调函数已调用！");
    // printf("%d %d",next_lf_degree,next_ud_degree);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"vision_arm_control");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("DJ_position",100);  //直接发布出去让另一个节点去控制舵机
    ros::Subscriber sub = nh.subscribe("goal_center_pixel_coordinate",100, offset_callback);
    std_msgs::Float32MultiArray head_position;

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        head_position.data.resize(3);
        head_position.data[0] = next_lf_degree;
        head_position.data[1] = next_ud_degree;
        head_position.data[2] = next_fb_degree;
        printf("%f %f %f\n",head_position.data[0] ,head_position.data[1] ,head_position.data[2]);

        pub.publish(head_position);
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}


