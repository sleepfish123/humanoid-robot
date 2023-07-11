/**
 * 该程序将订阅/apple_info和/head_theta 及通过运算给出各关节角度量
 */

#include <ros/ros.h>  
#include"apple_ik/joint_value.h"
#include <math.h>
#include <iostream>
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <boost/thread.hpp>
#include <sstream>
#include "std_msgs/String.h"
#include <Eigen/Dense>
#include<Eigen/Core> 
#include<Eigen/SVD>   
#include "geometry_msgs/PoseStamped.h"
using namespace std;
using namespace Eigen;

//fun函数转化，
void fun(double M[12], double T14, double T24,double T34,double mark,double the1,double the2,double the3,double the4,double the5)
//（存放计算结果数组M，传入T2最后一列及标志）
 {
/*************************改******************************/
//double dd1=200, dd3=183, dd5=300, pi=3.1415926;
/*************************改******************************/
double dd1=221, dd3=195.6, dd5=300, pi=3.1415926;
/*************************改******************************/
 double cos1=cos(the1),cos2=cos(the2), cos3=cos(the3), cos4=cos(the4), cos5=cos(the5);
 double sin1=sin(the1), sin2=sin(the2), sin3=sin(the3),sin4=sin(the4), sin5=sin(the5);
double tt1[] = {cos1, -sin1, 0, 0,       sin1 ,cos1 ,0 ,0,       0 , 0, 1 , dd1,            0 ,0, 0, 1};
double tt2[] = {-sin2, -cos2, 0, 0,     0 ,0 ,-1 ,0,                   cos2 ,-sin2,0, 0,      0 ,0, 0, 1};
double tt3[] = {-sin3, -cos3, 0, 0,     0 ,0 ,1 ,dd3,               -cos3 ,sin3,0, 0,      0 ,0, 0, 1};
double tt4[] = {sin4, cos4, 0, 0,        0 ,0 ,-1,0,                   -cos4 ,sin4,0, 0,       0 ,0, 0, 1};
double tt5[] = {cos5, -sin5, 0, 0,      0 ,0 ,-1 ,0,                  sin5 ,cos5,0, 0,         0 ,0, 0, 1};
double tran[] = {1, 0, 0, 0,     0 ,1, 0,0,   0, 0, 1,dd5,   0 ,0, 0, 1}; //%姿态不变换到和大地坐标系姿态一致
Matrix1 	TT1 = new_matrix1(tt1, 4, 4); 
Matrix1 	TT2 = new_matrix1(tt2, 4, 4);              
Matrix1 	TT3 = new_matrix1(tt3, 4, 4);
Matrix1 	TT4 = new_matrix1(tt4 ,4, 4);
Matrix1 	TT5 = new_matrix1(tt5, 4, 4);
Matrix1 	TRAN = new_matrix1(tran, 4, 4); 
Matrix1 MM1 = TT1 * TT2;
Matrix1 MM2 = MM1 * TT3;
Matrix1 MM3 = MM2 * TT4;
Matrix1 MM4 = MM3 * TT5;
Matrix1 MM5 = MM4 * TRAN; //  printf("MM5:" );   print_matrix1(	MM5);  
           double a11=0.5101; double a12=0.606; double a13=0.6103; 
           double a21=0.761;   double a22=0.01258; double  a23=-0.6486;
           double a31=-0.4007; double   a32= 0.7954; double  a33=-0.4548;
           double a14=T14; double   a24=T24; double  a34=T34;
if(mark==1){double a11=0.3907; double a12=0.2934; double a13=0.8725; 
           double a21 = 0.8694; double a22=0.1939; double  a23=-0.4545;
           double a31 = -0.3026; double   a32=0.9361; double  a33=-0.1793;
           double a14 = T14; double   a24=T24; double  a34=T34;}
double f11=MM5.data[0][0]-a11;
double f12=MM5.data[0][1]-a12;
double f13=MM5.data[0][2]-a13;
double f14=MM5.data[0][3]-a14;
double f21=MM5.data[1][0]-a21;
double f22=MM5.data[1][1]-a22;
double f23=MM5.data[1][2]-a23;
double f24=MM5.data[1][3]-a24;
double f31=MM5.data[2][0]-a31;
double f32=MM5.data[2][1]-a32;
double f33=MM5.data[2][2]-a33;
double f34=MM5.data[2][3]-a34;
M[0] = f11;M[1] = f12;M[2] = f13;M[3] = f14;M[4] = f21;M[5] = f22;
M[6] = f23;M[7] = f24;M[8] = f31;M[9] = f32;M[10] = f33;M[11] = f34;//m9有问题
 }
//dfun函数转化，
//void dfun(double M1[12], double M2[12],double M3[12],double M4[12],double M5[12],double mark,)
void dfun(double M1[12], double M2[12],double M3[12],double M4[12],double M5[12],double theta1,double theta2,double theta3,double theta4,double theta5)
//（存放计算结果数组M）
 {
/*************************改******************************/
//double dd1=200, dd3=183, dd5=300, pi=3.1415926;
/*************************改******************************/
double dd1=221, dd3=195.6, dd5=300, pi=3.1415926;
/*************************改******************************/
double m11=-1.0*sin(theta5)*(cos(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2)) - 1.0*cos(theta5)*(sin(theta4)*(cos(theta1)*cos(theta3) + 1.0*sin(theta1)*sin(theta2)*sin(theta3)) + 1.0*cos(theta2)*cos(theta4)*sin(theta1));
double  m12=sin(theta5)*(sin(theta4)*(cos(theta1)*cos(theta3) + 1.0*sin(theta1)*sin(theta2)*sin(theta3)) + 1.0*cos(theta2)*cos(theta4)*sin(theta1)) - 1.0*cos(theta5)*(cos(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2));
double  m13=cos(theta4)*(cos(theta1)*cos(theta3) + 1.0*sin(theta1)*sin(theta2)*sin(theta3)) - cos(theta2)*sin(theta1)*sin(theta4);
double  m14=195.6*cos(theta2)*sin(theta1) + 300.0*cos(theta4)*(cos(theta1)*cos(theta3) + 1.0*sin(theta1)*sin(theta2)*sin(theta3)) - 300.0*cos(theta2)*sin(theta1)*sin(theta4);
double  m15=- sin(theta5)*(sin(theta1)*sin(theta3) + 1.0*cos(theta1)*cos(theta3)*sin(theta2)) - cos(theta5)*(sin(theta4)*(cos(theta3)*sin(theta1) - cos(theta1)*sin(theta2)*sin(theta3)) - cos(theta1)*cos(theta2)*cos(theta4));
double  m16=1.0*sin(theta5)*(sin(theta4)*(cos(theta3)*sin(theta1) - cos(theta1)*sin(theta2)*sin(theta3)) - cos(theta1)*cos(theta2)*cos(theta4)) - cos(theta5)*(sin(theta1)*sin(theta3) + 1.0*cos(theta1)*cos(theta3)*sin(theta2));
double  m17=1.0*cos(theta4)*(cos(theta3)*sin(theta1) - cos(theta1)*sin(theta2)*sin(theta3)) + cos(theta1)*cos(theta2)*sin(theta4);
double  m18=300.0*cos(theta4)*(cos(theta3)*sin(theta1) - cos(theta1)*sin(theta2)*sin(theta3)) - 195.6*cos(theta1)*cos(theta2) + 300.0*cos(theta1)*cos(theta2)*sin(theta4);
double  m19=0;double  m110=0, m111=0, m112=0;
M1[0] = m11;M1[1] = m12;M1[2] = m13;M1[3] = m14;M1[4] = m15;M1[5] = m16;M1[6] = m17;M1[7] = m18;M1[8] = m19;M1[9] = m110;M1[10] = m111;M1[11] = m112;
double  m20=- 1.0*cos(theta5)*(1.0*cos(theta1)*cos(theta4)*sin(theta2) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)*sin(theta4)) - 1.0*cos(theta1)*cos(theta2)*cos(theta3)*sin(theta5) ;
double  m21= sin(theta5)*(1.0*cos(theta1)*cos(theta4)*sin(theta2) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)*sin(theta4)) - 1.0*cos(theta1)*cos(theta2)*cos(theta3)*cos(theta5);
double  m22=- cos(theta1)*sin(theta2)*sin(theta4) - 1.0*cos(theta1)*cos(theta2)*cos(theta4)*sin(theta3) ;
double  m23=195.6*cos(theta1)*sin(theta2) - 300.0*cos(theta1)*sin(theta2)*sin(theta4) - 300.0*cos(theta1)*cos(theta2)*cos(theta4)*sin(theta3) ;
double  m24=- cos(theta5)*(cos(theta4)*sin(theta1)*sin(theta2) - cos(theta2)*sin(theta1)*sin(theta3)*sin(theta4)) - 1.0*cos(theta2)*cos(theta3)*sin(theta1)*sin(theta5) ;
double  m25=1.0*sin(theta5)*(cos(theta4)*sin(theta1)*sin(theta2) - cos(theta2)*sin(theta1)*sin(theta3)*sin(theta4)) - 1.0*cos(theta2)*cos(theta3)*cos(theta5)*sin(theta1) ;
double  m26=- sin(theta1)*sin(theta2)*sin(theta4) - 1.0*cos(theta2)*cos(theta4)*sin(theta1)*sin(theta3) ;
double  m27=195.6*sin(theta1)*sin(theta2) - 300.0*sin(theta1)*sin(theta2)*sin(theta4) - 300.0*cos(theta2)*cos(theta4)*sin(theta1)*sin(theta3) ;
double  m28=cos(theta5)*(cos(theta2)*cos(theta4) + 1.0*sin(theta2)*sin(theta3)*sin(theta4)) - cos(theta3)*sin(theta2)*sin(theta5) ;
double  m29=- 1.0*sin(theta5)*(cos(theta2)*cos(theta4) + 1.0*sin(theta2)*sin(theta3)*sin(theta4)) - cos(theta3)*cos(theta5)*sin(theta2) ;
double  m210=cos(theta2)*sin(theta4) - cos(theta4)*sin(theta2)*sin(theta3) ;
double  m211=300.0*cos(theta2)*sin(theta4) - 195.6*cos(theta2) - 300.0*cos(theta4)*sin(theta2)*sin(theta3) ;
M2[0] = m20;M2[1] = m21;M2[2] = m22;M2[3] = m23;M2[4] = m24;M2[5] = m25;M2[6] = m26;
M2[7] = m27;M2[8] = m28;M2[9] = m29;M2[10] = m210;M2[11] = m211;
double  m30=1.0*cos(theta5)*sin(theta4)*(sin(theta1)*sin(theta3) + 1.0*cos(theta1)*cos(theta3)*sin(theta2)) - 1.0*sin(theta5)*(cos(theta3)*sin(theta1) - cos(theta1)*sin(theta2)*sin(theta3));
double  m31=- 1.0*cos(theta5)*(cos(theta3)*sin(theta1) - cos(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*sin(theta5)*(sin(theta1)*sin(theta3) + 1.0*cos(theta1)*cos(theta3)*sin(theta2));
double  m32=-cos(theta4)*(sin(theta1)*sin(theta3) + 1.0*cos(theta1)*cos(theta3)*sin(theta2));
double  m33=-300.0*cos(theta4)*(sin(theta1)*sin(theta3) + 1.0*cos(theta1)*cos(theta3)*sin(theta2));
double  m34= sin(theta5)*(cos(theta1)*cos(theta3) + 1.0*sin(theta1)*sin(theta2)*sin(theta3)) - cos(theta5)*sin(theta4)*(cos(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2));
double  m35=  cos(theta5)*(cos(theta1)*cos(theta3) + 1.0*sin(theta1)*sin(theta2)*sin(theta3)) + 1.0*sin(theta4)*sin(theta5)*(cos(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2));
double  m36= 1.0*cos(theta4)*(cos(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2));
double  m37=  300.0*cos(theta4)*(cos(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2));
double  m38= - cos(theta2)*sin(theta3)*sin(theta5) - 1.0*cos(theta2)*cos(theta3)*cos(theta5)*sin(theta4);
double  m39=	 1.0*cos(theta2)*cos(theta3)*sin(theta4)*sin(theta5) - cos(theta2)*cos(theta5)*sin(theta3);;
double  m310=	 cos(theta2)*cos(theta3)*cos(theta4);
double  m311=	  300.0*cos(theta2)*cos(theta3)*cos(theta4);
M3[0] = m30;M3[1] = m31;M3[2] = m32;M3[3] = m33;M3[4] = m34;M3[5] = m35;M3[6] = m36;
M3[7] = m37;M3[8] = m38;M3[9] = m39;M3[10] = m310;M3[11] = m311;
double  m40=-1.0*cos(theta5)*(cos(theta4)*(cos(theta3)*sin(theta1) - 1.0*cos(theta1)*sin(theta2)*sin(theta3)) + 1.0*cos(theta1)*cos(theta2)*sin(theta4));
double  m41=sin(theta5)*(cos(theta4)*(cos(theta3)*sin(theta1) - 1.0*cos(theta1)*sin(theta2)*sin(theta3)) + 1.0*cos(theta1)*cos(theta2)*sin(theta4));
double  m42=cos(theta1)*cos(theta2)*cos(theta4) - sin(theta4)*(cos(theta3)*sin(theta1) - 1.0*cos(theta1)*sin(theta2)*sin(theta3));
double  m43=300.0*cos(theta1)*cos(theta2)*cos(theta4) - 300.0*sin(theta4)*(cos(theta3)*sin(theta1) - 1.0*cos(theta1)*sin(theta2)*sin(theta3));
double  m44= cos(theta5)*(cos(theta4)*(cos(theta1)*cos(theta3) + sin(theta1)*sin(theta2)*sin(theta3)) - cos(theta2)*sin(theta1)*sin(theta4));
double  m45=  -1.0*sin(theta5)*(cos(theta4)*(cos(theta1)*cos(theta3) + sin(theta1)*sin(theta2)*sin(theta3)) - cos(theta2)*sin(theta1)*sin(theta4));
double  m46= 1.0*sin(theta4)*(cos(theta1)*cos(theta3) + sin(theta1)*sin(theta2)*sin(theta3)) + cos(theta2)*cos(theta4)*sin(theta1);
double  m47= 300.0*sin(theta4)*(cos(theta1)*cos(theta3) + sin(theta1)*sin(theta2)*sin(theta3)) + 300.0*cos(theta2)*cos(theta4)*sin(theta1);	
double  m48=  -cos(theta5)*(sin(theta2)*sin(theta4) + 1.0*cos(theta2)*cos(theta4)*sin(theta3));
double  m49=  1.0*sin(theta5)*(sin(theta2)*sin(theta4) + 1.0*cos(theta2)*cos(theta4)*sin(theta3));
double  m410=  cos(theta4)*sin(theta2) - cos(theta2)*sin(theta3)*sin(theta4);
double  m411=  300.0*cos(theta4)*sin(theta2) - 300.0*cos(theta2)*sin(theta3)*sin(theta4);
M4[0] = m40;M4[1] = m41;M4[2] = m42;M4[3] = m43;M4[4] = m44;M4[5] = m45;M4[6] = m46;
M4[7] = m47;M4[8] = m48;M4[9] = m49;M4[10] = m410;M4[11] = m411;
double  m50=1.0*sin(theta5)*(sin(theta4)*(cos(theta3)*sin(theta1) - 1.0*cos(theta1)*sin(theta2)*sin(theta3)) - 1.0*cos(theta1)*cos(theta2)*cos(theta4)) - 1.0*cos(theta5)*(sin(theta1)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)); 
double  m51=cos(theta5)*(sin(theta4)*(cos(theta3)*sin(theta1) - 1.0*cos(theta1)*sin(theta2)*sin(theta3)) - 1.0*cos(theta1)*cos(theta2)*cos(theta4)) + 1.0*sin(theta5)*(sin(theta1)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2));
double  m52=0;double  m53=0; 
double  m54=cos(theta5)*(cos(theta1)*sin(theta3) - 1.0*cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta5)*(sin(theta4)*(cos(theta1)*cos(theta3) + sin(theta1)*sin(theta2)*sin(theta3)) + cos(theta2)*cos(theta4)*sin(theta1));
double  m55=- sin(theta5)*(cos(theta1)*sin(theta3) - 1.0*cos(theta3)*sin(theta1)*sin(theta2)) - 1.0*cos(theta5)*(sin(theta4)*(cos(theta1)*cos(theta3) + sin(theta1)*sin(theta2)*sin(theta3)) + cos(theta2)*cos(theta4)*sin(theta1));
double  m56=0;double  m57= 0;
double  m58= cos(theta2)*cos(theta3)*cos(theta5) - sin(theta5)*(cos(theta4)*sin(theta2) - 1.0*cos(theta2)*sin(theta3)*sin(theta4));
double  m59= - 1.0*cos(theta5)*(cos(theta4)*sin(theta2) - 1.0*cos(theta2)*sin(theta3)*sin(theta4)) - cos(theta2)*cos(theta3)*sin(theta5);
double  m510= 0;double  m511= 0;
M5[0] = m50;M5[1] = m51;M5[2] = m52;M5[3] = m53;M5[4] = m54;M5[5] = m55;M5[6] = m56;
M5[7] = m57;M5[8] = m58;M5[9] = m59;M5[10] = m510;M5[11] = m511;
}
//	ros回调函数发布消息：把函数写成类的形式，把需要的一些变量在类中声明为全局变量。
double xc=0;double yc=0;double zc=0; int single1=0, single2=0,single3=0;
int head1=0; double th1=0,th2=0,mark=0;
void revalue(double x, double y,double z)//此处接受由订阅1传来的苹果位置
{   //此处为了验证方便直接赋值
	xc=x;yc=y;zc=z;//***************************************************************************************************//
//xc=21.4538;;yc=120.367;zc=353;
single1=1;
}
int h=0;
class SubscribeAndPublish  
{  
public:  
int num=0;
int count1=0;int count2=0,se1=0;
  SubscribeAndPublish()  
  {  
    pub_ = n_.advertise<apple_ik::joint_value>("right_arm_value", 1000);  
    sub_ = n_.subscribe("apple/pose", 1000, &SubscribeAndPublish::callback1, this); //订阅摄像头重新看到苹果的信息
   // sub2_ = n_.subscribe("/head_theta", 1000, &SubscribeAndPublish::callback2, this); 
	sub3_ = n_.subscribe("/head_over", 1000, &SubscribeAndPublish::callback3, this); 
	//ap_position=nh.advertise<image_process::apple_coordinate>("apple/positions",1000);
  }  
/******************************************************************************************/
  void callback1(const geometry_msgs::PoseStamped::ConstPtr& msg1)  
  {  
  if(se1==0){
double    a2= msg1->pose.position.x;
double     b2= msg1->pose.position.y;
 double    c2=  msg1->pose.position.z; 
   //double    a2= msg1->px;            double     b2= msg1->py;    double     c2= msg1->pz; 
   revalue( a2,b2,c2);
  }
   int mark2=99;
  ros::Rate loop_rate(100);
count1++;
loop_rate.sleep();
single1=10;
  }  
/******************************************************************************************/

  void callback3(const apple_ik::joint_value::ConstPtr& msg3)
  {
	if(num==0)
{
num=1;
sleep(1);h=100;ros::Rate loop_rate(100);
double tht1=msg3->theta1;                       //获取到头部关节
double tht2=msg3->theta2; int mark= msg3->mark;  //获取到头部关节

//添加FABRIK求解的上半身的解
// th111=60*pii/180; th222=-10*pii/180;th333=-30*pii/180;th444=-10*pii/180; th555=-10*pii/180; 


//printf("头部动作完成，重新获取苹果位置为xc:%f ,yc:%f ,zc:%f ,mark:%d\n ",xc,yc,zc,mark);
int i=1,j=1;
while(zc>600)
{
	for(int j=0;j<10;j++)
	{
		loop_rate.sleep();
	}
	printf("waiting new site! ");
}
while(zc<60)
{
	for(int j=0;j<10;j++)
	{
		loop_rate.sleep();
	}
 	printf("new site false! ");
 }
se1=100;
double eps=0.01;int false1=100;
th1=tht1;th2=tht2;
double  theta1= th1*3.1415926/180;  double theta2= th2*3.1415926/180;
printf("*******%d:机器人头部动作角度为 theta1:%f theta2:%f  ，目标位置标志（目标位置高0）为mark1:%d，当前摄像头重新获取苹果位置为 xc:%f  yc;%f   zc %f:\n", count2,th1,th2, mark,xc,yc,zc);
double s1=sin(theta1);
double s2=sin(theta2);
double c1=cos(theta1);
double c2=cos(theta2);

/***************************改1********************************************/
//double x0=32.5;double z0=68.5;double h0=230;
/***************************改1********************************************/
double x0=32.5;double z0=68.5;double h0=190;
/***************************改1********************************************/
double t0[] = { 1,0 ,0, h0,   0 ,1 ,0 ,0,   0 ,0,1, 0,    0 ,0, 0, 1};
double r1[] = {c1, -s1, 0, 0,   s1 ,c1 ,0 ,0,   0 ,0,1, 0,    0 ,0, 0, 1};
double r2[] = {1,0 ,0, 0,   0, c2,-s2,0  , 0, s2, c2,0,    0,0,0, 1};
double tt[] = {0, -1, 0, 0,   0 ,0, -1,-z0,   1, 0, 0, -x0,   0 ,0, 0, 1};
Matrix1 	T0 = new_matrix1(t0, 4, 4); 
Matrix1 	R1 = new_matrix1(r1, 4, 4);              
Matrix1 	R2 = new_matrix1(r2, 4, 4);
Matrix1 	TT = new_matrix1(tt, 4, 4);

/***************************改1********************************************/
//double px=- yc+230;double py= -zc-68.5;double pz=xc-32.5;
/***************************改1********************************************/
double px=- yc+190;double py= -zc-68.5;double pz=xc-32.5;
/***************************改1********************************************/
double ca[]={1,0,0,xc,    0,1,0,yc, 0,0,1,zc,  0, 0, 0,1};
Matrix1 	camera = new_matrix1(ca, 4, 4);
Matrix1 M1=matrixmultiply1(T0, R1);
Matrix1 M2=matrixmultiply1(M1, R2);
Matrix1 M3=matrixmultiply1(M2, TT);
Matrix1 T2=matrixmultiply1(M3, camera);
double T14=T2.data[0][3];
double T24=T2.data[1][3];
double T34=T2.data[2][3];
printf("矩阵t2:" );   print_matrix1(T2);  
double *M = new double[12];   double *MA = new double[12];  
double *MB = new double[12];  double *MC = new double[12];  
double *MD = new double[12];  double *ME = new double[12];  
double aq1, aq2, aq3,aq4, aq5,TH[5], THT[5];
double th111,th222,th333, th444, th555, pii=3.1415926;
double  d, f4,f8, f12, MM;int k, times=1;
Eigen::MatrixXd xn(5,12);  Eigen::MatrixXd x1(1,12);  Eigen::MatrixXd xnn1;

for( k=1;k<900;k++){//开始迭代****************************************************************

fun( M, T14,  T24, T34, mark,th111,th222,th333,th444,th555);
dfun( MA, MB,  MC, MD,ME, th111,th222,th333,th444,th555);
for(int i=0;i<12;i++){xn(0,i)=MA[i];}  
for(int i=0;i<12;i++){xn(1,i)=MB[i];}
for(int i=0;i<12;i++){xn(2,i)=MC[i];}
for(int i=0;i<12;i++){xn(3,i)=MD[i];}
for(int i=0;i<12;i++){xn(4,i)=ME[i];} 
for(int i=0;i<12;i++){x1(0,i)=M[i];}  
 xnn1=x1*pseudoInverse(xn);
 aq1=xnn1(0,0);  aq2=xnn1(0,1);  aq3=xnn1(0,2);  aq4=xnn1(0,3);  aq5=xnn1(0,4);
  TH[5];TH[0]=aq1;TH[1]=aq2;TH[2]=aq3;TH[3]=aq4;TH[4]=aq5;
THT[0]=th111-TH[0];THT[1]=th222-TH[1];THT[2]=th333-TH[2];THT[3]=th444-TH[3];THT[4]=th555-TH[4];
for(int j=0;j<5;j++){    d=THT[j];THT[j]= fmod(d,6.2831852);      }//求余运算
 f4=M[3]; f8=M[7]; f12=M[11]; MM=f4*f4+f8*f8+f12*f12;
th111=THT[0]; th222=THT[1];th333=THT[2];;th444=THT[3];; th555=THT[4];
 if(k==660)    
     {
	printf(" 迭代出错 \n");k=999; 
    false1=100;
	}
if(MM < eps&&THT[1]/pii*180>-130&&THT[2]/pii*180<10&&THT[4]/pii*180>-50&&THT[4]/pii*180<120)    
     {
    printf(" 迭代成功，");printf(" 所需次数：%d， ",times);
    k=999;false1=0;
    }
times++;
}//迭代结束***********************************************************
if(false1==100)
{
printf(" 迭代失败\n");

//初始值为FABRIK计算出来的上半身的解
  double th11,th22,th33,th44,th55;
   th11=60; th22=-10;th33=-30;th44=-10; th55=-10;   //订阅的消息进行赋值

apple_ik::joint_value right1;
right1.theta1=th11;   right1.theta2=th22;   right1.theta3=th33;  right1.theta4=th44;  right1.theta5=th55;
ros::Rate loop_rate(10);
loop_rate.sleep();
printf("迭代失败发布机械臂关节角度："); printf(" th1:%f,th2:%f,th3:%f,th4:%f,th5:%f ", th11,th22,th33,th44,th55);
loop_rate.sleep();
pub_.publish(right1);
loop_rate.sleep();
pub_.publish(right1);
loop_rate.sleep();
pub_.publish(right1);
}
else{
double z=180/3.1415926, e=1;
printf("迭代成功得到的机械臂关节角度："); 
for (int g = 0; g < 5; g++) {   e=THT[g];e=e*z; printf("  %f ", e);   }  
printf("\n");
apple_ik::joint_value right;
double a1,a2,a3,a4,a5;a1= THT[0]*z;a2= THT[1]*z;a3= THT[2]*z;a4= THT[3]*z;a5= THT[4]*z;
right.theta1=a1;   right.theta2=a2;   right.theta3=a3;  right.theta4=a4;  right.theta5=a5;
  if(a1>=-60&&a1<=120&&  a2>=-60&&a2<=91&&  a3>=-90&&a3<=91&& a4>=-90&&a4<=60&&a5>=-90&&a5<=91)

{
	pub_.publish(right);
	loop_rate.sleep();
    pub_.publish(right);
    loop_rate.sleep();
    pub_.publish(right);
printf("迭代成功发布机械臂关节角度："); printf(" th1:%f,th2:%f,th3:%f,th4:%f,th5:%f ", a1,a2,a3,a4,a5);
}
else{
	printf("迭代结果为不可达点，关节角度："); printf(" th1:%f,th2:%f,th3:%f,th4:%f,th5:%f ", a1,a2,a3,a4,a5);
	double th111,th222,th333,th444,th555;
	 if(mark==1)
   {
   th111=60; th222=-10;th333=-30;th444=-10; th555=-10;
   } 
else
     {
	th111=110; th222=-5;th333=-20;th444=-45; th555=0;
	 }
apple_ik::joint_value right2;
right2.theta1=th111;   right2.theta2=th222;   right2.theta3=th333;  right2.theta4=th444;  right2.theta5=th555;
ros::Rate loop_rate(100);
	
	
pub_.publish(right2);
loop_rate.sleep();
pub_.publish(right2);
loop_rate.sleep();
pub_.publish(right2);
}
	count2++;
	}


}
else {printf("*");}

}

 // void callback2(const apple_site::theta::ConstPtr& msg2); 
private:  
  ros::NodeHandle n_;   
  ros::Publisher pub_;  
  ros::Subscriber sub_;
  ros::Subscriber sub2_; 
  ros::Subscriber sub3_;
  std_msgs::String output;
};//End of class SubscribeAndPublish  

int main(int argc, char **argv)  
{  
	//Initiate ROS  
	ros::init(argc, argv, "robotarm3");  
	SubscribeAndPublish test;  
	//ros::spin();
	ros::MultiThreadedSpinner s(2);  //多线程

	ros::spin(s);  
}  
