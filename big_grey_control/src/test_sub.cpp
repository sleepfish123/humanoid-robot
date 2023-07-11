/*订阅FABRIK计算出来的下半身各关节解和目标苹果的位置信息 "joint_message"  */
/*订阅头部转动时扫描得到苹果的位置信息 "ros_message"  */
/*发布头部转动角度到底层  */
/*发布头部转动角度和扫描得到苹果的位置信息到第二次求解器 */

#include <ros/ros.h>
#include <test_message/test_msg.h>
#include <iostream>
#include <math.h>
#include <test_message/joint_value.h>

#define PI   3.1415
int n_value=0;
double theta8=0,theta9=0;

using namespace std;
int x=0;
double X_apple=0,Y_apple=0, Z_apple=0;   
double X_head=0, Y_head=0, Z_head=0;  //摄像头坐标系相对于基坐标系的位置信息
double a3=186, a7=275+230, d4=545;    //a7表示腰部平台中心到摄像头平面中心的距离
double x0=32.5, z0=68.5;
 double theta31=0,theta41=0,theta61=0,theta71=0;
double dx1=0,dy1=0,hz1=0;
int num=1;
int x_1=0,y_1=0,z_1=0;
double c3=0,s3=0,c4=0,s4=0,c6=0,s6=0,c7=0,s7=0;

void Callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    x_1 = msg->data.at(0);
    y_1 = msg->data.at(1);
    z_1 = msg->data.at(2);
       //cout<<"x_1 = "<< x_1<<endl;
}

void jointCallback(const sensor_msgs::JointState::ConstPtr& joint_msg){


        dx1=joint_msg->dx;  dy1= joint_msg->dy; hz1= joint_msg->hz;  theta31= joint_msg->theta3; 
        theta41= joint_msg->theta4;   theta61= joint_msg->theta6;theta71= joint_msg->theta7;
        X_apple=joint_msg->apple_x; Z_apple=joint_msg->apple_z;

        c3=cos(theta31*PI/180), s3=sin(theta31*PI/180);
        c4=cos(theta41*PI/180), s4=sin(theta41*PI/180);
        c6=cos(theta61*PI/180), s6=sin(theta61*PI/180);
        c7=cos(theta71*PI/180), s7=sin(theta71*PI/180);

        X_head=dx1-x0*(c7*(c3*c4-s3*s4)+s6*s7*(c3*s4+c4*s3))-a3*s3-a7*(s7*(c3*c4-s3*s4)-c7*s6*(c3*s4+c4*s3))-c6*z0*(c3*s4+c4*s3);
        Y_head=dy1+a3*c3-x0*(c7*(c3*s4+c4*s3)-s6*s7*(c3*c4-s3*s4))-a7*(s7*(c3*s4+c4*s3)+c7*s6*(c3*c4-s3*s4))+c6*z0*(c3*c4-s3*s4);
        Z_head=d4+hz1+s6*z0+a7*c6*c7-c6*s7*x0;

        //cout<<"X_Y_Z_head=="<<X_head<<" "<<Y_head<<" "<<Z_head<<" "<<endl;

        if((X_head<=X_apple)&& Z_head<=Z_apple){
            n_value=1;
            //cout<<"n_num = "<< n_value<<endl;
        }
            
        if((X_head>X_apple)&& Z_head<=Z_apple){
            n_value=2;
            //cout<<"n_num  = "<<n_value<<endl;
        }

        if((X_head>X_apple)&& Z_head>Z_apple){
            n_value=3;
            //cout<<"n_num  = "<<n_value<<endl;
        }

        if((X_head<=X_apple)&& Z_head>Z_apple){
            n_value=4;
            //cout<<"n_num  = "<<n_value<<endl;
        }

       switch (n_value){
            case 1:
            {
                    theta8=theta8+2*num;
                    theta9=theta9-6*num;
                    num ++;
                    //发布头部转动
                    //cout<<"theta8 _number  = "<<theta8<<endl;
                    //cout<<"theta9 _number  = "<<theta9<<endl;

                break;
            }

            case 2:
            {
                    theta8=theta8+2*num;
                    theta9=theta9+6*num;
                    num++;
                    //发布头部转动
                    //cout<<"theta8 _number  = "<<theta8<<endl;
                    //cout<<"theta9 _number  = "<<theta9<<endl;            

                break;
            }

            case 3:
            {
                    theta8=theta8-2*num;
                    theta9=theta9+6*num;
                    num++;
                    //发布头部转动
                    //cout<<"theta8 _number  = "<<theta8<<endl;
                    //cout<<"theta9 _number  = "<<theta9<<endl;              

                break;
            }

            case 4:
            {
                    theta8=theta8-2*num;
                    theta9=theta9-6*num;
                    num++;
                    //发布头部转动
                    //cout<<"theta8 _number  = "<<theta8<<endl;
                    //cout<<"theta9 _number  = "<<theta9<<endl;        

                break;
            }

            default:
            cout<<"ERROR!"<<endl;       
        }



}

int main(int argc, char **argv){
    ros::init(argc, argv, "topic_subscriber");
     ros::NodeHandle nh_;
     ros::Rate loop_rate(10);

     ros::Subscriber sub=nh_.subscribe("ros_message",100, Callback); //订阅头部转动时获取到的苹果信息
     ros::Subscriber sub_joint=nh_.subscribe("joint_message",100, jointCallback); // 订阅FABRIK求解出来的下半身各关节和第一次看到的苹果位置信息

     while(ros::ok() && (theta8>=-20) && (theta8<=40) && (theta9>=-90) && (theta9<=90)){

     if((x_1>=-10) && (x_1<=10) && (y_1>=-10) && (y_1<=10) && (z_1>0)){

      //发布x_1 y_1 z_1 theta8 theta9  
     }

         ros::spinOnce();
         loop_rate.sleep();

     }
}
