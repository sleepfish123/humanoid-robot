#include<ros/ros.h>
#include<iostream>
#include<std_msgs/Float32MultiArray.h>
using namespace std;



float offset_dead_block = 30;// 设置偏移量死区
float next_lf_degree,next_ud_degree;
float last_lf_degree=0,last_ud_degree=0;
float u0 = 320,v0 = 240;  //假设图像中心像素坐标为(0,0);      image_depth_rgb.cpp中查阅  图像中心点像素坐标为（320，240）

//定义 左右移动舵机控制函数
float lf_servo(float offset_x)
{
    float lf_kp=-0.0088;
    float delta_degree;

    //设置调节死区
    if(abs(offset_x)<offset_dead_block)
     {
        offset_x = 0;
     }
    delta_degree = lf_kp * offset_x;
    next_lf_degree = last_lf_degree + delta_degree;
    

    //限幅
    if(next_lf_degree < -60)
        next_lf_degree = -60;
    else if (next_lf_degree > 60)
        next_lf_degree = 60;

    last_lf_degree = next_lf_degree;
     return next_lf_degree;   
  
}

//定义 上下移动舵机控制函数
float ud_servo(float offset_y)
{
    float ud_kp = -0.0065;
    float delta_degree;

    //设置调节死区
    if(abs(offset_y)<offset_dead_block)
        offset_y = 0;

    delta_degree = ud_kp * offset_y; 
    next_ud_degree = last_ud_degree + delta_degree;
    

    //限幅
    if(next_ud_degree < -25)
        next_ud_degree = -25;
    else if (next_ud_degree > 25)
        next_ud_degree = 25;

    last_ud_degree = next_ud_degree;
     return next_ud_degree;   
}

void offset_callback(std_msgs::Float32MultiArray goal_uv)
{
    
    float offset[2]={0,0};

    offset[0] = goal_uv.data[0] - u0;
    offset[1] = goal_uv.data[1] - v0;

    next_lf_degree = lf_servo(offset[0]);
    next_ud_degree = ud_servo(offset[1]);
    // ROS_INFO("回调函数已调用！");
    // printf("%d %d",next_lf_degree,next_ud_degree);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"vision_head_control");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("DJ_position",10);  //直接发布出去让另一个节点去控制舵机，还是直接在这个节点写控制？
    ros::Subscriber sub = nh.subscribe("goal_center_pixel_coordinate",10, offset_callback);
    std_msgs::Float32MultiArray head_position;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        head_position.data.resize(2);
        head_position.data[0] = next_lf_degree;
        head_position.data[1] = next_ud_degree;
        printf("%f %f\n",head_position.data[0] ,head_position.data[1] );

        pub.publish(head_position);
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}


