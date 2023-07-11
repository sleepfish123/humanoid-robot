#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include "big_grey_control/image.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float32.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "big_grey_control/big_grey_srv.h"

// 舵机控制头文件
#include <unistd.h> // 延时函数头文件 sleep(5); // 延迟5秒
// #include "image_process/servo_command.h"

#define fx 617.0245971679688       // 定义了焦距
#define fy 617.330810546875        // 定义了y方向上的焦距
#define cx 320.0            //定义了图片u方向上的 中心坐标值
#define cy 240.0            //定义了图片v方向上的中心点坐标值
using namespace std;
using namespace cv;

class depth_rgb
{
// private:

public:
  ros::NodeHandle nh;
  image_transport::ImageTransport it;                 //定义imagetransport类型的句柄、其目的是加快图片的传输速率
  image_transport::Subscriber rgb_sub,depth_sub,aligned_sub;   //定义了彩色、深度、对其图像的订阅、格式为image-transport
  ros::Subscriber box_sub,boxnum_sub;                 
  ros::Publisher ap_position;                          //定义信息发布
  ros::Publisher chatter_pub;
  ros::Publisher boundingbox_value_pub;
  ros::Publisher applesite_pub;
  ros::Publisher applesitemark_pub;

  ros::Publisher  pixel_pub;     //发布苹果在图像中的像素坐标

   int object_number=0;
   float x_min[100]={0},x_max[100]={0},y_min[100]={0},y_max[100]={0},c_x[100]={0},c_y[100]={0};     //定义了bounding_box 的四个角的坐标数组以及中心点坐标数组；
   float X_ap[100]={0},Y_ap[100]={0},Z_ap[100]={0};                      //定义检测出来的苹果的坐标数组
   float ap_proba[100]={0};                            //定义了识别目标的概率数组
   //string object_class[100];                       //定义了识别目标的类别   
   geometry_msgs::PoseStamped msg1;
   std_msgs::Float32MultiArray apple_site_msg;
   std_msgs::Bool apple_site_mark_msg;

   std_msgs::Float32MultiArray goal_uv;   //苹果像素坐标（u，v）
   std_msgs::Float32MultiArray goal_uvd;   //苹果像素坐标（u，v） 加上深度坐标d 
   
    //定义了苹果消息类型
   int number;
    
    depth_rgb():it(nh)
    {
    /***********************订阅检测到目标的个数消息******************************************************/
     // boxnum_sub=nh.subscribe("/darknet_ros/found_object",1,&depth_rgb::boxnum_cb,this);

    /***********************订阅检测到目标的boundingbox 的消息******************************************************/
      box_sub=nh.subscribe("/darknet_ros/bounding_boxes",100,&depth_rgb::box_cb,this);

     /***********************订阅摄像的发布的彩色图像、深度图像、对其图像的消息******************************************************/
      // rgb_sub=it.subscribe("/camera/color/image_raw",1,&depth_rgb::rgb_cb,this);
      //  depth_sub=it.subscribe("/camera/depth/image_rect_raw",1,&depth_rgb::depth_cb,this);
      // aligned_sub=it.subscribe("/cam_1/aligned_depth_to_color/image_raw",1,&depth_rgb::aligned_cb,this);
      aligned_sub=it.subscribe("/camera/aligned_depth_to_color/image_raw",1,&depth_rgb::aligned_cb,this);

     /***********************发布检测目标的位置消息******************************************************/
    //  ap_position=nh.advertise<image_process::apple_coordinate>("apple/positions",1000); 
     applesitemark_pub=nh.advertise<std_msgs::Bool>("apple_site_mark",1000); 
     applesite_pub=nh.advertise<std_msgs::Float32MultiArray>("apple_site",1000);
     chatter_pub = nh.advertise<geometry_msgs::PoseStamped>("apple/pose", 1000); 
     pixel_pub=nh.advertise<std_msgs::Float32MultiArray>("goal_center_pixel_coordinate",100);   //发布苹果像素坐标
    }

     ~depth_rgb()
     {

     }

/****************彩色图片读取（消息转换过程）*****************************/
void rgb_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat rgb_image;
    try
    {
        rgb_image = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8)->image;
        //imageencoding里面定义了图像对应于消息的编码格式，如rgb8 彩色图片  type—16uc1 深度图
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge execption : %s",e.what());
        return;
    }

     cv::imshow("rgb_image",rgb_image);
     cv::waitKey(10);
} 


/****************深度图片读取***********************************/
void depth_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat depth_image;
    try
    {
        depth_image = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1)->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return ;
    }
    cv::imshow("depth_image",depth_image);
    cv::waitKey(5);
    ROS_INFO("%f",depth_image.at<float>(depth_image.rows/2,depth_image.cols/2));


    /***********************两种读取深度图中某点的深度的方式***************************************/
    // ROS_INFO("%f",depth_image.at<float>(cv::Point(320,240)));
    // cout<<"ddddddddddd"<<depth_image.at<float>(depth_image.rows/2,depth_image.cols/2)<<endl;
}


/***********************对齐图片操作***************************************/
void aligned_cb(const sensor_msgs::ImageConstPtr& msg)
{
    ros::Rate loop_rate(10);
    cv::Mat depth_rgb_image;
    try  {
        depth_rgb_image = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1)->image;
    }
    catch(cv_bridge::Exception& e)  {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }
 //   cv::imshow("depth_rgb",depth_rgb_image);
    cv::waitKey(5);
    for(int i=0;i<object_number&&i<1;i++)
    // for(int i=0;i<1;i++)
    {
        // double sumdepth=0;
        // for(int m=x_min[i];m<x_max[i]+1;m++)
        // {
        //     for(int n=y_min[i];n<y_max[i]+1;n++)
        //    {
        //       sumdepth=(depth_rgb_image.at<float>(cv::Point(c_x[i],c_y[i])));   //获取世界坐标的z值
        //     }
        // }
       Z_ap[i]=(depth_rgb_image.at<float>(cv::Point(c_x[i],c_y[i])));   //获取世界坐标的z值
       cout<<"the distance of apple is   "<<Z_ap[i]<<endl;
       X_ap[i]=((c_x[i]-cx)*Z_ap[i]/fx);     
                                  //获取世界坐标的x值
       Y_ap[i]=((c_y[i]-cy)*Z_ap[i]/fy);                                //获取世界坐标的y值
       cout<<"z======="<<Z_ap[i]<<endl;
       cout<<"x======="<<X_ap[i]<<endl;
       cout<<"y======="<<Y_ap[i]<<endl;

    // std_msgs::Float32 boundingbox_y;
    // // boundingbox_x.data = (x_min[0]+x_max[0]) / 2;
    // boundingbox_y.data = Y_ap[i];
    // boundingbox_value_pub = nh.advertise<std_msgs::Float32>("/boundngbox_value", 1000); 
    // boundingbox_value_pub.publish(boundingbox_y);
    std_msgs::Float32 boundingbox_x;
    // boundingbox_x.data = (x_min[0]+x_max[0]) / 2;
    boundingbox_x.data = X_ap[i];
    boundingbox_value_pub = nh.advertise<std_msgs::Float32>("/boundngbox_value", 1000);

    boundingbox_value_pub.publish(boundingbox_x);


   // ros::Time currentTime = ros::Time::now();
    //msg1.header.stamp = currentTime;
    msg1.pose.position.x = X_ap[i];
    msg1.pose.position.y = Y_ap[i];
    msg1.pose.position.z =Z_ap[i];

    apple_site_msg.data.resize(3);
    apple_site_msg.data[0]=X_ap[i];
    apple_site_msg.data[1]=Y_ap[i];
    apple_site_msg.data[2]=Z_ap[i];

    goal_uv.data.resize(2);
    goal_uv.data[0] = c_x[i];
    goal_uv.data[1] = c_y[i];

    goal_uvd.data.resize(3);
    goal_uvd.data[0]=c_x[i];
    goal_uvd.data[1]=c_y[i];
    goal_uvd.data[2]=Z_ap[i];

    // cout<<"goal_uvd.data[3]======="<<Z_ap[i]<<endl;
    // cout<<"goal_uvd.data[3]======="<<goal_uvd.data[3]<<endl;


    if(Y_ap[i]>0){
        apple_site_mark_msg.data=1;
    }
    else apple_site_mark_msg.data=0;

    double a=Z_ap[i];
    // apple_msg.objectclass=object_class[i];
    if(a>100&&a<1600)
    {
    // ap_position.publish(apple_msg);
    // chatter_pub.publish(msg1);   //原始函数发送的
    applesitemark_pub.publish(apple_site_mark_msg);
    applesite_pub.publish(apple_site_msg);
    pixel_pub.publish(goal_uvd);
    }
    // ros::spinOnce();
     loop_rate.sleep();
    }
}

void box_cb(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
    /****************读取boundbox的类别，概率等 的方式****************************/
    // cout<<"header:  "<<msg->header<<endl;
    // cout<<"image_header:  "<<msg->image_header<<endl;
    // cout<<"Class:  "<<msg->bounding_boxes[0].Class<<endl;
    // cout<<"bounding box xmin:  "<<msg->bounding_boxes[0].xmin<<endl;
    // cout<<"bounding box xmax:  "<<msg->bounding_boxes[0].ymin<<endl;
    // cout<<"probability:  "<<msg->bounding_boxes[0].probability<<endl;

    int count = msg->bounding_boxes.size();
    for(int i=0;i<count;i++)
    {
        cout<<"number===    "<<count<<endl;
        object_number=count;
        x_min[i]=(msg->bounding_boxes[i].xmin);   //获取boundingbox的四个角的坐标xmin
        x_max[i]=(msg->bounding_boxes[i].xmax);   //获取boundingbox的四个角的坐标xmax
        y_min[i]=(msg->bounding_boxes[i].ymin);   //获取boundingbox的四个角的坐标ymin
        y_max[i]=(msg->bounding_boxes[i].ymax);    //获取boundingbox的四个角的坐标ymax
   
       cout<<"bounding box xmin:  "<<x_min[i]<<"......"<<y_min[i]<<endl;
       cout<<"bounding box xmax:  "<<x_max[i]<<"......"<<y_max[i]<<endl;

       c_x[i]=((x_min[i]+x_max[i])/2);            //获取中心点的坐标c_x
       c_y[i]=((y_min[i]+y_max[i])/2);            //获取中心点的坐标c_y
       ap_proba[i]=(msg->bounding_boxes[i].probability);//获取该目标的检测概率
       //object_class[i]=(msg->bounding_boxes[i].Class);  //获取检测目标的类别
    }
}
};

 int main(int argc, char *argv[])
{
    ros::init(argc, argv, "depth_rgb");
    ros::NodeHandle nh;
    depth_rgb dr;
    ros::spin();
    return 0;
}

