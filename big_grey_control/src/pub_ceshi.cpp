#include<ros/ros.h>
#include<std_msgs/String.h>
#include<std_msgs/Int8MultiArray.h>
#include<std_msgs/Float32MultiArray.h>
#include<iostream>
using namespace std;

int main(int argc, char  *argv[])
{
    ros::init(argc,argv,"pub_ceshi");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("goal_center_pixel_coordinate",100);

    std_msgs::Float32MultiArray goal_uv;
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        goal_uv.data.resize(3);
        goal_uv.data[0] = 401;
        goal_uv.data[1] = 159;
        goal_uv.data[2] = 120;
        // printf("开始发布目标像素坐标");
        pub.publish(goal_uv);
        
        printf("%f %f %f\n",goal_uv.data[0],goal_uv.data[1],goal_uv.data[2]);
        
        
        loop_rate.sleep();
    }
    
    return 0;
}

