#include <robot_collision_checking/fcl_interface.h>
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <vector>

// 发布ros消息的相关头文件
#include "std_msgs/Float32MultiArray.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <ctime> // 使用里面的计时函数

using namespace std;
using namespace Eigen;

#define pi 3.1415926536

int flag_finish = 0;

/*********************************************************大灰各部分参数定义**************************************************/
double foot_to_ground = 0.476; // 266.5 + 182.34 = 448.84mm
double waist_to_foot = 0.652; // 641.94
double waist_to_chest = 0.362; // 355
double chest_to_shoulder = 0.223; // 220.9
double shoulder_to_elbow = 0.193; // 195.8
double elbow_to_gripper = 0.287; // 297.08
double car_to_foot = 0.13299; // 132.99
double headCenter_to_chest = 0.152; // 

double waist_to_ground = foot_to_ground + waist_to_foot; // 0.476+0.652=1.128
double chest_to_ground = waist_to_ground + waist_to_chest; // 1.128+0.362=1.49
double headCenter_to_ground = chest_to_ground + headCenter_to_chest; // 1.49+0.152=1.642

// 单位m
Eigen::Vector3d octo_position(0.0, 0.0, 0.0); // octo
Eigen::Vector3d pw_position(0.0, car_to_foot, waist_to_ground); // pw
Eigen::Vector3d body_position(0.0, car_to_foot, (chest_to_ground - waist_to_chest / 2)); // 身体
Eigen::Vector3d p1_position(0.0, car_to_foot, chest_to_ground); // p1
Eigen::Vector3d shoulder_position(chest_to_shoulder / 2, car_to_foot, chest_to_ground); // 肩部
Eigen::Vector3d p2_position(chest_to_shoulder, car_to_foot, chest_to_ground); // p2
Eigen::Vector3d big_arm_position(chest_to_shoulder, car_to_foot, (chest_to_ground - shoulder_to_elbow / 2)); // 大臂
Eigen::Vector3d p3_position(chest_to_shoulder, car_to_foot, (chest_to_ground - shoulder_to_elbow)); // p3
Eigen::Vector3d small_arm_position(chest_to_shoulder, car_to_foot + elbow_to_gripper / 2, (chest_to_ground - shoulder_to_elbow)); // 小臂
Eigen::Vector3d p4_position(chest_to_shoulder, car_to_foot + elbow_to_gripper, (chest_to_ground - shoulder_to_elbow)); // p4（0.2209,0.43007,1.24998）

Eigen::Vector3d half_p3_p4; // p3和p4的中点
Eigen::Vector3d half_p2_p3; // p2和p3的中点
Eigen::Vector3d half_p1_p2; // p1和p2的中点
Eigen::Vector3d p2p1_direction_vector, p2p3_direction_vector;

int segmentation = 2;

int flag_backward;

Eigen::Matrix3d octo_rotation, pw_rotation, body_rotation, p1_rotation, shoulder_rotation, p2_rotation, big_arm_rotation, p3_rotation, small_arm_rotation, p4_rotation;
Eigen::Matrix3d cylinder_rotation;

Eigen::Affine3d octo, pw_Affine3d, body, p1_Affine3d, shoulder, p2_Affine3d, big_arm, p3_Affine3d, small_arm, p4_Affine3d;

shape_msgs::SolidPrimitive sphere_pw, cylinder_body, sphere_p1, cylinder_shoulder, sphere_p2, cylinder_big_arm, sphere_p3, cylinder_small_arm, sphere_p4;

octomap_msgs::Octomap octomap_;

/************************************************************************************************************************/

/***************************************************** FABRIK变量定义 ***************************************************/
// FABRIK
double apple_site_x, apple_site_y, apple_site_z;

Eigen::Vector3d p_start = p4_position; // 起点坐标为p4（机械臂夹爪中心）的坐标
Eigen::Vector3d p1_start = p1_position;

Eigen::Vector3d cylinder_position;

int collision; // 碰撞检测标志位

// 定义方向向量
Eigen::Vector3d p3p2_direction_vector;
Eigen::Vector3d p3p4_direction_vector;

double body_length = waist_to_chest;
double shoulder_length = chest_to_shoulder;
double big_arm_length = shoulder_to_elbow;
double small_arm_length = elbow_to_gripper;

double d1 = chest_to_shoulder;
double d2 = shoulder_to_elbow;
double d3 = elbow_to_gripper;

double radius = 0.05; // 圆柱半径0.05m
double l; // 距离值

int failedAttempts = 0;
int maxFailedAttempts = 1000; // 最大错误次数
double forward_range = 0.03; // 前向后p1的活动范围
double backward_range = 0.001; // 后向后p4的范围

int random_number;
int random_angle;

double rad; // 弧度值
double cosrad_1, rad_1, theta_1; // 右肘部角度、弧度
double cosrad_2, rad_2, theta_2; // 侧抬角度、弧度

// 各弧度、角度定义
double rad_jiaobu; double theta_jiaobu;
double rad_wanyao; double theta_wanyao;
double rad_ceyao; double theta_ceyao;
// double rad_o; double theta_o;
double rad_qianhoutai; double theta_qianhoutai;
double rad_cetai; double theta_cetai;
double rad_dabi; double theta_dabi;
double rad_zhoubu; double theta_zhoubu;
double up_down_waist;
double car_qianhou; double car_zuoyou;

Eigen::Matrix3d Rx, Ry, Rz; // 绕x、y、z轴旋转的旋转矩阵
Eigen::Vector3d pw_xyz_, pw_xyz;

// 原先的各关节点位置
Eigen::Vector3d pw_0, pw;
Eigen::Vector3d p1_0, p1_1, p1;
Eigen::Vector3d p2_0, p2_1, p2;
Eigen::Vector3d p3_0, p3_1, p3;
Eigen::Vector3d p4_0, p4_1, p4;

int flag = 0;
int flag_apple_site = 0;

// 弧度值转角度值函数
double rad2deg(double rad_) {
    double theta_ = rad_ / pi * 180;
    return theta_;
}

// 角度值转弧度值函数
double deg2rad(double theta_) {
    double rad_ = theta_ / 180 * pi;
    return rad_;
}

Eigen::Matrix4d calculate_T9G(Eigen::Vector3d pw, double c6, double s6, double c7, double s7, double co, double so, double c8, double s8, double c9, double s9) {
    // 计算TwG
    Eigen::Matrix4d TwG;
    TwG << 1, 0, 0, pw[0], 0, 1, 0, pw[1], 0, 0, 1, pw[2], 0, 0, 0, 1;
    // 计算T4w
    Eigen::Matrix4d T4w;
    T4w << co, -so, 0, 0, so, co, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    // 计算T54
    Eigen::Matrix4d T54;
    T54 << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕zw旋转90°
    // 计算T65
    Eigen::Matrix4d T1;
    T1 << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1; // 绕x5旋转90°
    Eigen::Matrix4d T2;
    T2 << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z5旋转90°
    Eigen::Matrix4d T3;
    T3 << c6, -s6, 0, 0, s6, c6, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z5旋转θ6
    Eigen::Matrix4d T65 = T1 * T2 * T3;
    // 计算T76
    T1 << 1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1; // 绕x6旋转-90°
    T2 << c7, -s7, 0, 0, s7, c7, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z6旋转θ7
    Eigen::Matrix4d T76 = T1 * T2;
    // 计算To7
    Eigen::Matrix4d trans;
    trans << 1, 0, 0, body_length, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 沿x7平移300
    T1 << 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1; // 绕y7旋转90°
    // T2 << co, -so, 0, 0, so, co, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z7旋转θo
    Eigen::Matrix4d To7 = trans * T1;
    // 计算T8o
    trans << 1, 0, 0, 0, 0, 1, 0, -shoulder_length, 0, 0, 1, 0, 0, 0, 0, 1; // 沿yo平移-110
    T1 << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕zo旋转90°
    T2 << 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1; // 绕yo旋转-90°
    T3 << c8, -s8, 0, 0, s8, c8, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕zo旋转θ8
    Eigen::Matrix4d T8o = trans * T1 * T2 * T3;
    // 计算T98
    T1 << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z8旋转90°
    T2 << 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1; // 绕y8旋转-90°
    T3 << c9, -s9, 0, 0, s9, c9, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z8旋转θ9
    Eigen::Matrix4d T98 = T1 * T2 * T3;

    Eigen::Matrix4d T9G = TwG * T4w * T54 * T65 * T76 * To7 *  T8o * T98;

    return T9G;
}

Eigen::Matrix4d calculate_ToG(Eigen::Vector3d pw, double c6, double s6, double c7, double s7, double co, double so) {
    // 计算TwG
    Eigen::Matrix4d TwG;
    TwG << 1, 0, 0, pw[0], 0, 1, 0, pw[1], 0, 0, 1, pw[2], 0, 0, 0, 1;
    // 计算T4w
    Eigen::Matrix4d T4w;
    T4w << co, -so, 0, 0, so, co, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    // 计算T54
    Eigen::Matrix4d T54;
    T54 << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕zw旋转90°
    // 计算T65
    Eigen::Matrix4d T1;
    T1 << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1; // 绕x5旋转90°
    Eigen::Matrix4d T2;
    T2 << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z5旋转90°
    Eigen::Matrix4d T3;
    T3 << c6, -s6, 0, 0, s6, c6, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z5旋转θ6
    Eigen::Matrix4d T65 = T1 * T2 * T3;
    // 计算T76
    T1 << 1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1; // 绕x6旋转-90°
    T2 << c7, -s7, 0, 0, s7, c7, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z6旋转θ7
    Eigen::Matrix4d T76 = T1 * T2;
    // 计算To7
    Eigen::Matrix4d trans;
    trans << 1, 0, 0, body_length, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 沿x7平移300
    T1 << 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1; // 绕y7旋转90°
    // T2 << co, -so, 0, 0, so, co, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z7旋转θo
    Eigen::Matrix4d To7 = trans * T1;

    Eigen::Matrix4d ToG = TwG * T4w * T54 * T65 * T76 * To7;

    return ToG;
}

// 计算距离函数，计算三维向量a和b之间的距离
double distanceCost3(Eigen::Vector3d a, Eigen::Vector3d b) {
    double distance = sqrt(pow(a[0]-b[0], 2) + pow(a[1]-b[1], 2) + pow(a[2]-b[2], 2));
    return distance;
}

bool octomap_received ( false ), lin_callback_received(false);

/************************************* 碰撞检测函数 *************************************/
// 检测圆柱和octomap的碰撞
// 传入圆柱的高度(height)、半径(radius)、位置(position)、旋转姿态(rotation)
// void collision_detection(shape_msgs::SolidPrimitive *cylinder_1, Eigen::Affine3d *cylinder, double cylinder_height, double cylinder_radius, Eigen::Vector3d cylinder_position, Eigen::Matrix3d cylinder_rotation) {
//     // FCLInterface test_node ( nh );
//     ROS_INFO ( "FABRIK FCL" );
//     // ros::Duration(5.0).sleep();  

//     *cylinder.linear() = cylinder_rotation;
//     *cylinder.translation() = cylinder_position;

//     *cylinder_1.dimensions.resize ( 2 );
//     *cylinder_1.type=shape_msgs::SolidPrimitive::CYLINDER;
//     *cylinder_1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = cylinder_height;
//     *cylinder_1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = cylinder_radius;
// }

double calculate_jiaobu(Eigen::Vector3d p1_jb, Eigen::Vector3d p2_jb) {
    double rad_jiaobu_jb;
    Eigen::Vector3d p1p2_jb = p2 - p1;
    Eigen::Vector3d y_jb(0.0, 1.0, 0.0);
    Eigen::Vector3d z_jb(0.0, 0.0, 1.0);
    Eigen::Vector3d n1_jb = p1p2_jb.cross(z_jb);
    Eigen::Vector3d n2_jb = y_jb;
    rad_jiaobu_jb = acos(fabs(n1_jb.dot(n2_jb)) / (sqrt(n1_jb.dot(n1_jb)) * sqrt(n2_jb.dot(n2_jb))));
    return rad_jiaobu_jb;
}

Eigen::Vector2d calculate_dabi_zhoubu(Eigen::Vector3d p2, Eigen::Vector3d p3, Eigen::Vector3d p4, Eigen::Vector4d pp) {
    Eigen::Vector3d p2p3 = p3 - p2;
    Eigen::Vector3d n = p2p3;
    Eigen::Vector3d p3p4 = p4 - p3;
    Eigen::Vector3d n1 = n.cross(p3p4);
    Eigen::Vector3d p3p4_touying = n.cross(n1);
    Eigen::Vector3d l;
    l[0] = pp[0] - p3[0];
    l[1] = pp[1] - p3[1];
    l[2] = pp[2] - p3[2];
    
    // 求解大臂旋转角度
    double rad_dabi = acos(fabs(l.dot(p3p4_touying)) / (sqrt(l.dot(l)) * sqrt(p3p4_touying.dot(p3p4_touying))));
    
    // 求解肘部角度
    double rad_zhoubu = acos(fabs(p3p4.dot(p3p4_touying)) / (sqrt(p3p4.dot(p3p4)) * sqrt(p3p4_touying.dot(p3p4_touying))));
    
    Eigen::Vector2d rad_dabi_zhoubu;
    rad_dabi_zhoubu << rad_dabi, rad_zhoubu;

    return rad_dabi_zhoubu;
}


Eigen::Vector2d calculate_qianhoutai_cetai(Eigen::Vector3d pw, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3) {
    Eigen::Vector3d p1pw = pw - p1;
    Eigen::Vector3d p1p2 = p2 - p1;
    Eigen::Vector3d p2p3 = p3 - p2;

    Eigen::Vector3d n = p1p2;
    Eigen::Vector3d n1 = p2p3.cross(n);
    Eigen::Vector3d p2p3_touying = n.cross(n1);

    // 求前后抬角度
    double rad_qianhoutai = acos(fabs(p1pw.dot(p2p3_touying)) / (sqrt(p1pw.dot(p1pw)) * sqrt(p2p3_touying.dot(p2p3_touying))));
    
    // 求侧抬角度
    double rad_cetai = acos(fabs(p2p3.dot(p2p3_touying)) / (sqrt(p2p3.dot(p2p3)) * sqrt(p2p3_touying.dot(p2p3_touying))));
    
    // // 求小车旋转角度θo
    // Eigen::Vector3d x;
    // x << 1, 0, 0;
    // Eigen::Vector3d n2 = p1pw.cross(x);
    // Eigen::Vector3d n3 = p1p2.cross(n2);
    // Eigen::Vector3d p1p2_touying = n2.cross(n3);
    // double rad_o = acos(fabs(p1p2.dot(p1p2_touying)) / (sqrt(p1p2.dot(p1p2)) * sqrt(p1p2_touying.dot(p1p2_touying))));
    
    Eigen::Vector2d rad_qianhoutai_cetai;
    rad_qianhoutai_cetai << rad_qianhoutai, rad_cetai;

    return rad_qianhoutai_cetai;
}

Eigen::Vector2d calculate_wanyao_ceyao(Eigen::Vector3d pw, Eigen::Vector3d p1, double rad_jiaobu) {
    Eigen::Vector3d pwp1 = p1 - pw;
    Eigen::Vector3d n1;
    n1 << cos(rad_jiaobu), sin(rad_jiaobu), 0;
    Eigen::Vector3d n2 = pwp1.cross(n1);
    Eigen::Vector3d pwp1_touying = n1.cross(n2);
    Eigen::Vector3d z;
    z << 0, 0, 1;
    // cos(rad_ceyao) = abs(dot(pwp1,pwp1_touying)) / (sqrt(sum(pwp1.^2))*sqrt(sum(pwp1_touying.^2)));
    double rad_wanyao = acos(fabs(z.dot(pwp1_touying)) / (sqrt(z.dot(z)) * sqrt(pwp1_touying.dot(pwp1_touying))));
    // double theta_ceyao = rad2deg(rad_ceyao);
    double rad_ceyao = acos(fabs(pwp1.dot(pwp1_touying)) / (sqrt(pwp1.dot(pwp1)) * sqrt(pwp1_touying.dot(pwp1_touying))));
    // double theta_wanyao = rad2deg(rad_wanyao);
    Eigen::Vector2d rad_wanyao_ceyao_;
    rad_wanyao_ceyao_ << rad_wanyao, rad_ceyao;

    return rad_wanyao_ceyao_;
}

Eigen::Matrix4d calculate_T7G(Eigen::Vector3d pw, double c6, double s6, double c7, double s7, double co, double so) {
    // 计算TwG
    Eigen::Matrix4d TwG;
    TwG << 1, 0, 0, pw[0], 0, 1, 0, pw[1], 0, 0, 1, pw[2], 0, 0, 0, 1;
    // 计算T4w
    Eigen::Matrix4d T4w;
    T4w << co, -so, 0, 0, so, co, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    // 计算T54
    Eigen::Matrix4d T54;
    T54 << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕zw旋转90°
    // 计算T65
    Eigen::Matrix4d T1;
    T1 << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1; // 绕x5旋转90°
    Eigen::Matrix4d T2;
    T2 << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z5旋转90°
    Eigen::Matrix4d T3;
    T3 << c6, -s6, 0, 0, s6, c6, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z5旋转θ6
    Eigen::Matrix4d T65 = T1 * T2 * T3;
    // 计算T76
    T1 << 1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1; // 绕x6旋转-90°
    T2 << c7, -s7, 0, 0, s7, c7, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z6旋转θ7
    Eigen::Matrix4d T76 = T1 * T2;

    Eigen::Matrix4d T7G = TwG * T4w * T54 * T65 * T76;

    return T7G;
}

Eigen::Matrix4d calculate_T11G(Eigen::Vector3d pw, double c6, double s6, double c7, double s7, double co, double so, 
                                                                        double c8, double s8, double c9, double s9, double c10, double s10, double c11, double s11) {
    // 计算TwG
    Eigen::Matrix4d TwG;
    TwG << 1, 0, 0, pw[0], 0, 1, 0, pw[1], 0, 0, 1, pw[2], 0, 0, 0, 1;
    // 计算T4w
    Eigen::Matrix4d T4w;
    T4w << co, -so, 0, 0, so, co, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    // 计算T54
    Eigen::Matrix4d T54;
    T54 << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕zw旋转90°
    // 计算T65
    Eigen::Matrix4d T1;
    T1 << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1; // 绕x5旋转90°
    Eigen::Matrix4d T2;
    T2 << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z5旋转90°
    Eigen::Matrix4d T3;
    T3 << c6, -s6, 0, 0, s6, c6, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z5旋转θ6
    Eigen::Matrix4d T65 = T1 * T2 * T3;
    // 计算T76
    T1 << 1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1; // 绕x6旋转-90°
    T2 << c7, -s7, 0, 0, s7, c7, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z6旋转θ7
    Eigen::Matrix4d T76 = T1 * T2;
    // 计算To7
    Eigen::Matrix4d trans;
    trans << 1, 0, 0, body_length, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 沿x7平移300
    T1 << 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1; // 绕y7旋转90°
    // T2 << co, -so, 0, 0, so, co, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z7旋转θo
    Eigen::Matrix4d To7 = trans * T1;
    // 计算T8o
    trans << 1, 0, 0, 0, 0, 1, 0, -shoulder_length, 0, 0, 1, 0, 0, 0, 0, 1; // 沿yo平移-110
    T1 << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕zo旋转90°
    T2 << 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1; // 绕yo旋转-90°
    T3 << c8, -s8, 0, 0, s8, c8, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕zo旋转θ8
    Eigen::Matrix4d T8o = trans * T1 * T2 * T3;
    // 计算T98
    T1 << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z8旋转90°
    T2 << 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1; // 绕y8旋转-90°
    T3 << c9, -s9, 0, 0, s9, c9, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z8旋转θ9
    Eigen::Matrix4d T98 = T1 * T2 * T3;

    // 计算T109
    trans << 1, 0, 0, 0, 0, 1, 0, shoulder_to_elbow, 0, 0, 1, 0, 0, 0, 0, 1;  // 沿y9平移d3=183
    T1 << 1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1; // 绕x9旋转-90°
    T2 << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z9旋转90°
    T3 << c10, -s10, 0, 0, s10, c10, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z9旋转θ10
    Eigen::Matrix4d T109 = trans * T1 * T2 * T3;
    // 计算T1110
    T1 << 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1; // 绕y10旋转90°
    T2 << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1; // 绕x10旋转90°
    T3 << c11, -s11, 0, 0, s11, c11, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 绕z10旋转θ11
    Eigen::Matrix4d T1110 = T1 * T2 * T3;

    Eigen::Matrix4d T11G = TwG * T4w * T54 * T65 * T76 * To7 *  T8o * T98 * T109 * T1110;
    
    return T11G;
}

// 判断theta_qianhoutai、theta_cetai角度正负
vector<double> calculate_qianhoutai_cetai_vertify(Eigen::Vector3d p3, Eigen::Vector3d pw, 
                                                                                                                double theta_wanyao, double theta_ceyao, 
                                                                                                                double theta_jiaobu, 
                                                                                                                double theta_qianhoutai, double theta_cetai) {
    double rad6 = deg2rad(theta_wanyao); // 弯腰角度（前负）
    double rad7 = deg2rad(theta_ceyao); // 侧腰角度（右负）
    double c6 = cos(rad6); double s6 = sin(rad6);
    double c7 = cos(rad7); double s7 = sin(rad7);

    double rad_o = deg2rad(theta_jiaobu); // 小车旋转角度（右正）
    double co = cos(rad_o); double so = sin(rad_o);

    Eigen::Vector3d pw_abc = pw;
    double pw_x = pw_abc[0];
    double pw_y = pw_abc[1];
    double pw_z = pw_abc[2];

    Eigen::Vector4d p39;
    p39 << 0, shoulder_to_elbow, 0, 1; // p3相对于9坐标系的坐标
    Eigen::Vector4d p49;
    p49 << 0, shoulder_to_elbow, elbow_to_gripper, 1; // p4相对于9坐标系的坐标

    // 排列组合：(a, b)\(a, -b)\(-a, b)\(-a, -b)
    // ①(a, b)
    double rad8 = deg2rad(theta_qianhoutai); // 前后抬角度（前正）
    double rad9 = deg2rad(theta_cetai); // 侧抬角度（右负）
    double c8 = cos(rad8); double s8 = sin(rad8);
    double c9 = cos(rad9); double s9 = sin(rad9);
    Eigen::Matrix4d T9G = calculate_T9G(pw, c6, s6, c7, s7, co, so, c8, s8 ,c9, s9);
    Eigen::Vector4d p3G = T9G * p39; // p3=[301.00274, 446.33665, 1481.2104]
    Eigen::Vector3d p3G_3d;
    p3G_3d << p3G[0], p3G[1], p3G[2];
    // 计算(a, b)时计算出的p3和实际p3的距离
    double distance_p3_to_p3G_3d = distanceCost3(p3, p3G_3d);

    // 若距离大于0.001，则表示theta_qianhoutai和theta_cetai不对，尝试②(a, -b)
    if (distance_p3_to_p3G_3d > 0.01) {
        // 尝试②(a, -b)
        theta_cetai = -theta_cetai;
        rad8 = deg2rad(theta_qianhoutai); // 前后抬角度（前正）
        rad9 = deg2rad(theta_cetai); // 侧抬角度（右负）
        c8 = cos(rad8); s8 = sin(rad8);
        c9 = cos(rad9); s9 = sin(rad9);
        T9G = calculate_T9G(pw, c6, s6, c7, s7, co, so, c8, s8 ,c9, s9);
        p3G = T9G * p39; // p3=[301.00274, 446.33665, 1481.2104]
        p3G_3d << p3G[0], p3G[1], p3G[2];
        // 计算(a, b)时计算出的p3和实际p3的距离
        distance_p3_to_p3G_3d = distanceCost3(p3, p3G_3d);
        // 若距离大于0.001，则表示theta_qianhoutai和theta_cetai不对，尝试③(-a, -b)
        if (distance_p3_to_p3G_3d > 0.01) {
            theta_qianhoutai = -theta_qianhoutai;
            rad8 = deg2rad(theta_qianhoutai); // 前后抬角度（前正）
            rad9 = deg2rad(theta_cetai); // 侧抬角度（右负）
            c8 = cos(rad8); s8 = sin(rad8);
            c9 = cos(rad9); s9 = sin(rad9);
            T9G = calculate_T9G(pw, c6, s6, c7, s7, co, so, c8, s8 ,c9, s9);
            p3G = T9G * p39; // p3=[301.00274, 446.33665, 1481.2104]
            p3G_3d << p3G[0], p3G[1], p3G[2];
            // 计算(a, b)时计算出的p3和实际p3的距离
            distance_p3_to_p3G_3d = distanceCost3(p3, p3G_3d);
            // 若距离大于0.001，则表示theta_qianhoutai和theta_cetai不对，尝试④(-a, -b)
            if (distance_p3_to_p3G_3d > 0.01) {
                theta_cetai = -theta_cetai;
                rad8 = deg2rad(theta_qianhoutai); // 前后抬角度（前正）
                rad9 = deg2rad(theta_cetai); // 侧抬角度（右负）
                c8 = cos(rad8); s8 = sin(rad8);
                c9 = cos(rad9); s9 = sin(rad9);
                T9G = calculate_T9G(pw, c6, s6, c7, s7, co, so, c8, s8 ,c9, s9);
                p3G = T9G * p39; // p3=[301.00274, 446.33665, 1481.2104]
                p3G_3d << p3G[0], p3G[1], p3G[2];
                // 计算(a, b)时计算出的p3和实际p3的距离
                distance_p3_to_p3G_3d = distanceCost3(p3, p3G_3d);
                // 若①②③④都不行，则输出计算错误
                if (distance_p3_to_p3G_3d > 0.01) {
                    cout << "theta_qianhoutai和theta_cetai计算错误wuwuwuwuwuwu\n" << endl;
                }
            }
        }
    }
    vector<double> theta_qianhoutai_cetai_vertify;
    theta_qianhoutai_cetai_vertify.push_back(theta_qianhoutai);
    theta_qianhoutai_cetai_vertify.push_back(theta_cetai);
    return theta_qianhoutai_cetai_vertify;
}

// 判断theta_dabi、theta_zhoubu角度正负
vector<double> calculate_dabi_zhoubu_vertify(Eigen::Vector3d p_goal, Eigen::Vector3d pw, 
                                                                                                                double theta_wanyao, double theta_ceyao, 
                                                                                                                double theta_jiaobu, 
                                                                                                                double theta_qianhoutai, double theta_cetai, 
                                                                                                                double theta_dabi, double theta_zhoubu) {
    double rad6 = deg2rad(theta_wanyao); // 弯腰角度（前负）
    double rad7 = deg2rad(theta_ceyao); // 侧腰角度（右负）
    double c6 = cos(rad6); double s6 = sin(rad6);
    double c7 = cos(rad7); double s7 = sin(rad7);

    double rad_o = deg2rad(theta_jiaobu); // 小车旋转角度（右正）
    double co = cos(rad_o); double so = sin(rad_o);

    double rad8 = deg2rad(theta_qianhoutai); // 前后抬角度（前正）
    double rad9 = deg2rad(theta_cetai); // 侧抬角度（右负）
    double c8 = cos(rad8); double s8 = sin(rad8);
    double c9 = cos(rad9); double s9 = sin(rad9);

    Eigen::Vector3d pw_abc = pw;
    double pw_x = pw_abc[0];
    double pw_y = pw_abc[1];
    double pw_z = pw_abc[2];

    Eigen::Vector4d p411;
    p411 << 0, -elbow_to_gripper, 0, 1; // p4相对于11坐标系的坐标

    // 排列组合：(a, b)\(a, -b)\(-a, b)\(-a, -b)
    // ①(a, b)
    double rad10 = deg2rad(theta_dabi); // 大臂旋转角度（右zheng）
    double rad11 = deg2rad(theta_zhoubu);
    double c10 = cos(rad10); double s10 = sin(rad10);
    double c11 = cos(rad11); double s11 = sin(rad11);
    Eigen::Matrix4d T11G = calculate_T11G(pw, c6, s6, c7, s7, co, so, c8, s8 ,c9, s9, c10, s10, c11, s11);
    Eigen::Vector4d p4G = T11G * p411; // p4=[100.0, 550.0, 1450.0]
    Eigen::Vector3d p4G_3d;
    p4G_3d << p4G[0], p4G[1], p4G[2];
    // 计算theta_dabi为正时末端执行器和目标点之间的距离
    double distance_p_goal_to_p4G_3d = distanceCost3(p_goal, p4G_3d);
    // 若距离大于0.001，则表示theta_dabi和theta_zhoubu不对，尝试②(a, -b)
    if (distance_p_goal_to_p4G_3d > 0.01) {
        theta_zhoubu = -theta_zhoubu;
        rad10 = deg2rad(theta_dabi); // 大臂旋转角度（右zheng）
        rad11 = deg2rad(theta_zhoubu);
        c10 = cos(rad10); s10 = sin(rad10);
        c11 = cos(rad11); s11 = sin(rad11);
        T11G = calculate_T11G(pw, c6, s6, c7, s7, co, so, c8, s8 ,c9, s9, c10, s10, c11, s11);
        p4G = T11G * p411; // p4=[100.0, 550.0, 1450.0]
        p4G_3d << p4G[0], p4G[1], p4G[2];
        // 计算theta_dabi为正时末端执行器和目标点之间的距离
        distance_p_goal_to_p4G_3d = distanceCost3(p_goal, p4G_3d);
        // 若距离大于0.001，则表示theta_dabi和theta_zhoubu不对，尝试③(-a, -b)
        if (distance_p_goal_to_p4G_3d > 0.01) {
            theta_dabi = -theta_dabi;
            rad10 = deg2rad(theta_dabi); // 大臂旋转角度（右zheng）
            rad11 = deg2rad(theta_zhoubu);
            c10 = cos(rad10); s10 = sin(rad10);
            c11 = cos(rad11); s11 = sin(rad11);
            T11G = calculate_T11G(pw, c6, s6, c7, s7, co, so, c8, s8 ,c9, s9, c10, s10, c11, s11);
            p4G = T11G * p411; // p4=[100.0, 550.0, 1450.0]
            p4G_3d << p4G[0], p4G[1], p4G[2];
            // 计算theta_dabi为正时末端执行器和目标点之间的距离
            distance_p_goal_to_p4G_3d = distanceCost3(p_goal, p4G_3d);
            // 若距离大于0.001，则表示theta_dabi和theta_zhoubu不对，尝试④(-a, -b)
            if (distance_p_goal_to_p4G_3d > 0.01) {
                theta_zhoubu = -theta_zhoubu;       
                rad10 = deg2rad(theta_dabi); // 大臂旋转角度（右zheng）
                rad11 = deg2rad(theta_zhoubu);
                c10 = cos(rad10); s10 = sin(rad10);
                c11 = cos(rad11); s11 = sin(rad11);
                T11G = calculate_T11G(pw, c6, s6, c7, s7, co, so, c8, s8 ,c9, s9, c10, s10, c11, s11);
                p4G = T11G * p411; // p4=[100.0, 550.0, 1450.0]
                p4G_3d << p4G[0], p4G[1], p4G[2];
                // 计算theta_dabi为正时末端执行器和目标点之间的距离
                distance_p_goal_to_p4G_3d = distanceCost3(p_goal, p4G_3d);
                // 若①②③④都不行，则输出计算错误
                if (distance_p_goal_to_p4G_3d > 0.01) {
                    cout << "theta_dabi和theta_zhoubu计算错误wuwuwuwuwuwu\n" << endl;
                }
            }
        }
    }
    vector<double> theta_dabi_zhoubu_vertify;
    theta_dabi_zhoubu_vertify.push_back(theta_dabi);
    theta_dabi_zhoubu_vertify.push_back(theta_zhoubu);
    return theta_dabi_zhoubu_vertify;
}

// // 计算圆柱体旋转矩阵的函数
// Eigen::Matrix3d return_cylinder_rotation(Eigen::Vector3d pp3, Eigen::Vector3d pp4) {
//     Eigen::Vector3d p3p4 = pp4 - pp3;
//     Eigen::Matrix3d cylinder_rotation_;
//     Eigen::Vector3d z(0.0, 0.0, 1.0);
//     Eigen::Vector3d n(0.0, 1.0, 0.0);
//     Eigen::Vector3d n1 = p3p4.cross(n);
//     Eigen::Vector3d ll = n1.cross(n);
//     double rad_y = acos(fabs(ll.dot(z)) / (sqrt(ll.dot(ll)) * sqrt(z.dot(z))));
//     double rad_x = acos(fabs(ll.dot(p3p4)) / (sqrt(ll.dot(ll)) * sqrt(p3p4.dot(p3p4))));
//     Eigen::Matrix3d RY;
//     RY << cos(rad_y), 0, sin(rad_y),   0, 1, 0,   -sin(rad_y), 0, cos(rad_y);
//     Eigen::Matrix3d RX;
//     RX << 1, 0, 0,   0, cos(rad_x), -sin(rad_x),   0, sin(rad_x), cos(rad_x);
//     cylinder_rotation_ = RX * RY;
//     return cylinder_rotation_;
// }

void octoCallback ( const octomap_msgs::Octomap::ConstPtr& msg ) {
    octomap_=*msg;
    octomap_received=true;
    std::cout<<"received callback"<<std::endl;
    // flag_finish = 1;
}

void apple_site_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {    
    // cout << "待定x = " << msg->data.at(0) << endl;
    // cout << "待定y = " << msg->data.at(1) << endl;
    // cout << "待定z = " << msg->data.at(2) << endl;
    apple_site_x = msg->data.at(0);
    apple_site_y = msg->data.at(1);
    apple_site_z = msg->data.at(2);
    // cout << "apple_site_x：" << apple_site_x << endl;
    // cout << "apple_site_y：" << apple_site_y << endl;
    // cout << "apple_site_z：" << apple_site_z << endl;
    flag_apple_site = 1;
}

Eigen::Vector3d calculate_pw_xyz(Eigen::Vector3d p1_, Eigen::Vector3d p2_, Eigen::Vector3d pw_) {
    // Eigen::Vector3d p1(a1, b1, c1); // 这样定义的话，a、b为列向量
    // Eigen::Vector3d p2(a2, b2, c2);
    // Eigen::Vector3d pw(a, b, c);
    double a1, b1, c1;
    double a2, b2, c2;
    double a, b, c;
    double xx, yy, zz;
    double lamuda;
    double fenzi;
    double fenmu;

    Eigen::Vector3d pw_xyz_;

    a1 = p1_[0], b1 = p1_[1], c1 = p1_[2];
    a2 = p2_[0], b2 = p2_[1], c2 = p2_[2];
    a = pw_[0], b = pw_[1], c = pw_[2];

    fenzi = 2 * ((a-a1)*(a2-a1)+(b-b1)*(b2-b1)+(c-c1)*(c2-c1));
    fenmu = (a2-a1) * (a2-a1) + (b2-b1) * (b2-b1) + (c2-c1) * (c2-c1);
    lamuda = fenzi / fenmu;

    // 计算pw_xyz
    xx = (a1-a2) * lamuda / 2 + a;
    yy = (b1-b2) * lamuda / 2 + b;
    zz = (c1-c2) * lamuda / 2 + c;

    pw_xyz_[0] = xx;
    pw_xyz_[1] = yy;
    pw_xyz_[2] = zz;

    // Eigen::Vector3d half_p1_pw_xyz;
    // half_p1_pw_xyz = pw_xyz + (p1 - pw_xyz) / 2;

    return pw_xyz_;
}

int main ( int argc, char **argv ) {
    setlocale(LC_ALL,""); // 中文输出
    ros::init ( argc, argv, "octomap" ); // ros init
    ros::NodeHandle nh; // Create a node handle and start the node
    ros::Subscriber  octo_sub= nh.subscribe ( "/octomap_full", 1, &octoCallback );
    ros::Subscriber apple_site_sub = nh.subscribe("apple_site", 1000, apple_site_callback);

    std::cout<<"---------------------------------------------"<<std::endl;
    ROS_INFO ( "LABOUR FCL" );
    std::cout<<"---------------------------------------------"<<std::endl;

    // 等待接收到apple_site
    while (flag_apple_site == 0) {
        ROS_INFO("未接收到apple_site");
        ros::spinOnce();
        sleep(1);
    }

    // 将苹果坐标转化成苹果在世界坐标系下的坐标
    // 小车脚部中心向下与地的接触点，是世界坐标系的原点
    // 0.188，68.5，32.5
    double headCenter_to_camera_shangxia = 0.0385; // 单位m
    double headCenter_to_camera_zuoyou = 0.0325;
    double headCenter_to_camera_qianhou = 0.0685;
    double p_goal_x, p_goal_y, p_goal_z;
    // p_goal_x = apple_site_x - camera_to_eye_center;
    // p_goal_y = apple_site_z + camera_to_head_center + car_to_foot;
    // p_goal_z = -apple_site_y + camera_to_chest + chest_to_ground;

    p_goal_x = (double)apple_site_x / 1000 - headCenter_to_camera_zuoyou;
    p_goal_y = (double)apple_site_z / 1000 + car_to_foot + headCenter_to_camera_qianhou;
    p_goal_z = -(double)apple_site_y / 1000 + headCenter_to_ground + headCenter_to_camera_shangxia; // + 相机中心到地面
    // p_goal_x = 135.0 / 1000; // 注意：一定要加.0，否则135/1000=0
    // p_goal_y = 522.0 / 1000;
    // p_goal_z = 1333.0 / 1000; // + 头部中心到地面
    cout << "*****************************苹果相对于世界坐标系的位置*****************************" << endl;
    cout << "apple_site_x：" << apple_site_x << endl;
    cout << "apple_site_y：" << apple_site_y << endl;
    cout << "apple_site_z：" << apple_site_z << endl;
    cout << "p_goal_x：" << p_goal_x << endl;
    cout << "p_goal_y：" << p_goal_y << endl;
    cout << "p_goal_z：" << p_goal_z << endl;
    sleep(10); // **************************
    cout << "**************************************************************************************" << endl;
    // p_goal_x = apple_site_x;
    // p_goal_y = apple_site_y;
    // p_goal_z = apple_site_z;

    Eigen::Vector3d p_goal(p_goal_x, p_goal_y, p_goal_z); // 终点坐标为目标位姿
    ROS_INFO("接收到apple_site，算法开始");

    /******************************** 定义图形 ***********************************/
    sphere_pw.dimensions.resize ( 1 );
    sphere_pw.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere_pw.type=shape_msgs::SolidPrimitive::SPHERE;

    cylinder_body.dimensions.resize ( 2 );
    cylinder_body.type=shape_msgs::SolidPrimitive::CYLINDER;
    cylinder_body.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]=body_length;
    cylinder_body.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]=0.05;

    sphere_p1.dimensions.resize ( 1 );
    sphere_p1.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere_p1.type=shape_msgs::SolidPrimitive::SPHERE;

    cylinder_shoulder.dimensions.resize ( 2 );
    cylinder_shoulder.type=shape_msgs::SolidPrimitive::CYLINDER;
    cylinder_shoulder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]=shoulder_length;
    cylinder_shoulder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]=0.05;

    sphere_p2.dimensions.resize ( 1 );
    sphere_p2.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere_p2.type=shape_msgs::SolidPrimitive::SPHERE;

    cylinder_big_arm.dimensions.resize ( 2 );
    cylinder_big_arm.type=shape_msgs::SolidPrimitive::CYLINDER;
    cylinder_big_arm.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]=big_arm_length;
    cylinder_big_arm.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]=0.05;

    sphere_p3.dimensions.resize ( 1 );
    sphere_p3.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere_p3.type=shape_msgs::SolidPrimitive::SPHERE;

    cylinder_small_arm.dimensions.resize ( 2 );
    cylinder_small_arm.type=shape_msgs::SolidPrimitive::CYLINDER;
    cylinder_small_arm.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]=small_arm_length;
    cylinder_small_arm.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]=0.05;

    sphere_p4.dimensions.resize ( 1 );
    sphere_p4.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere_p4.type=shape_msgs::SolidPrimitive::SPHERE;
    
    /******************************** 绘制机械臂初始构型 ***********************************/
    Eigen::Vector3d p1_start_1 = p1_position;
    Eigen::Vector3d p2_start_1 = p2_position;
    Eigen::Vector3d p3_start_1 = p3_position;
    Eigen::Vector3d p4_start_1 = p4_position;

    p1 = p1_position;
    p2 = p2_position;
    p3 = p3_position;
    p4 = p4_position;
    pw = pw_position;

    octo_rotation.setIdentity();
    pw_rotation.setIdentity();
    body_rotation.setIdentity();
    p1_rotation.setIdentity();
    shoulder_rotation << 0, 0, 1,   0, 1, 0,   -1, 0, 0; // 肩部绕y轴转90度
    p2_rotation.setIdentity();
    big_arm_rotation.setIdentity();
    p3_rotation.setIdentity();
    small_arm_rotation << 1, 0, 0,   0, 0, -1,   0, 1, 0; // 小臂绕x轴转90度
    p4_rotation.setIdentity();

    octo.linear() = octo_rotation;
    octo.translation() = octo_position;
    pw_Affine3d.linear() = pw_rotation;
    pw_Affine3d.translation() = pw_position;
    body.linear() = body_rotation;
    body.translation() = body_position;
    p1_Affine3d.linear() = p1_rotation;
    p1_Affine3d.translation() = p1_position;
    shoulder.linear() = shoulder_rotation;
    shoulder.translation() = shoulder_position;
    p2_Affine3d.linear() = p2_rotation;
    p2_Affine3d.translation() = p2_position;
    big_arm.linear() = big_arm_rotation;
    big_arm.translation() = big_arm_position;
    p3_Affine3d.linear() = p3_rotation;
    p3_Affine3d.translation() = p3_position;
    small_arm.linear() = small_arm_rotation;
    small_arm.translation() = small_arm_position;
    p4_Affine3d.linear() = p4_rotation;
    p4_Affine3d.translation() = p4_position;

    FCLInterface test_node(nh);
    // ROS_INFO ( "Waiting a few seconds for RVIZ to start up" );
    // ros::Duration(5.0).sleep();    

    test_node.addCollisionObject ( sphere_pw, pw_Affine3d, 0 );
    test_node.addCollisionObject ( cylinder_body, body, 1 );
    test_node.addCollisionObject ( sphere_p1, p1_Affine3d, 2 );
    test_node.addCollisionObject ( cylinder_shoulder, shoulder, 3 );
    test_node.addCollisionObject ( sphere_p2, p2_Affine3d, 4 );
    test_node.addCollisionObject ( cylinder_big_arm, big_arm, 5 );
    test_node.addCollisionObject ( sphere_p3, p3_Affine3d, 6 );
    test_node.addCollisionObject ( cylinder_small_arm, small_arm, 7 );
    test_node.addCollisionObject ( sphere_p4, p4_Affine3d, 8 );
    test_node.displayObjects();
    test_node.removeCollisionObject(0);
    test_node.removeCollisionObject(1);
    test_node.removeCollisionObject(2);
    test_node.removeCollisionObject(3);
    test_node.removeCollisionObject(4);
    test_node.removeCollisionObject(5);
    test_node.removeCollisionObject(6);
    test_node.removeCollisionObject(7);
    test_node.removeCollisionObject(8);

    double negative_theta_1;
    double negative_theta_2;

    sleep(1); // 读octomap
    ros::spinOnce(); // 也可以用下面这一坨，但是
    // int i = 0;
    // while (i < 1) {
    //     i += 1;
    //     sleep(1); // 读octomap
    //     ros::spinOnce();
    // }
        
    /************************************************* 调用FABRIK ***********************************************************/
    if (distanceCost3(p1_start, p_goal)>(forward_range+d1+d2+d3)) {
        cout << "p1_start：" << p1_start << endl;
        cout << "p_goal：" << p_goal << endl;
        cout << forward_range+d1+d2+d3 << endl;
        printf("目标不可达 \n");
        sleep(10);
        ros::shutdown();
    }

    bool is_in_collision;
    shape_msgs::SolidPrimitive cylinder_1_;
    Eigen::Affine3d cylinder_;

    srand((int)time(0));  // 产生随机种子  把0换成NULL也行
    clock_t startTime, endTime;
    startTime = clock(); // 计时开始
    while (failedAttempts <= maxFailedAttempts) {
        random_number = 0;
        // sleep(1);
        printf("失败次数 = %d \n", failedAttempts); // 打印失败次数

        // 前向FABRIK
        // 以p_goal为目标点，FABRIK确定机械臂构型
        p4_0 = p4; // 将原先的p4记录下来
        p4 = p_goal; // 把p4拉到p_goal
        l = distanceCost3(p3, p4);
        cout << "111 l" << l << endl;
        p3_0 = p3; // 将原先的p3记录下来
        p3[0] = (l - d3) / l * p4[0] + d3 / l * p3[0];
        p3[1] = (l - d3) / l * p4[1] + d3 / l * p3[1];
        p3[2] = (l - d3) / l * p4[2] + d3 / l * p3[2];
        /***********************************************************/
        for (int iii1=0; iii1<(segmentation+1); iii1++) {
            cylinder_position = p3 + iii1 * (p4 - p3) / segmentation;
            cylinder_.linear() = pw_rotation;
            cylinder_.translation() = cylinder_position;
            cylinder_1_.dimensions.resize ( 1 );
            cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
            cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
            test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
            test_node.removeCollisionObject(0);
            test_node.removeCollisionObject(99 );
            test_node.addCollisionObject ( octomap_, octo, 99 );
            is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
            if (is_in_collision == 1) {
                break;
            }
        }
        /***********************************************************/
        if (is_in_collision) {
            collision = 1;
            cout << "11111111111111111111111111111111111111111111111111111111" << endl;
            // 有碰撞再生成随机数
            while (random_number == 0) {
                // 生成[a,b)范围内的随机整数，使用(rand() % (b-a))+ a;
                random_number = (rand() % (4-1)) + 1;
                cout << "111111111111111111111111111random_number：" << random_number << endl;
            }
            random_angle = (rand() % (361 - 1)) + 1; // 随机旋转角度，[1, 361)
            cout << "111111111111111111111111111random_angle：" << random_angle << endl;
        }
        else {
            collision = 0;
        }

        if (collision == 1) {
            cout << "22222222222222222222222222222222222222222222222" << endl;
            if (random_number == 1) {
                for (int i1=0; i1<1; i1++) { // 尝试50次随机旋转
                    rad = random_angle * pi / 180; // 转化为弧度值
                    Rx << 1, 0, 0,   0, cos(rad), -sin(rad),   0, sin(rad), cos(rad);
                    p3 = Rx * p3;
                    /***********************************************************/
                    for (int iii1=0; iii1<(segmentation+1); iii1++) {
                        cylinder_position = p3 + iii1 * (p4 - p3) / segmentation;
                        cylinder_.linear() = pw_rotation;
                        cylinder_.translation() = cylinder_position;
                        cylinder_1_.dimensions.resize ( 1 );
                        cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                        cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                        test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                        test_node.removeCollisionObject(0);
                        test_node.removeCollisionObject(99 );
                        test_node.addCollisionObject ( octomap_, octo, 99 );
                        is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                        if (is_in_collision == 1) {
                            break;
                        }
                    }
                    /***********************************************************/
                    if (is_in_collision) {
                        cout << "3333333333333333333333333333333333333333333333333333333" << endl;
                        continue;
                    }
                    else {
                        collision = 0; // 避开了障碍物，collision置零
                        break;
                    }
                }
            }
            else if (random_number == 2) {
                for (int i2=0; i2<1; i2++) { // 尝试50次随机旋转
                    rad = random_angle * pi / 180; // 转化为弧度值
                    Ry << cos(rad), 0, sin(rad),   0, 1, 0,   -sin(rad), 0, cos(rad);
                    p3 = Ry * p3;
                    /***********************************************************/
                    for (int iii1=0; iii1<(segmentation+1); iii1++) {
                        cylinder_position = p3 + iii1 * (p4 - p3) / segmentation;
                        cylinder_.linear() = pw_rotation;
                        cylinder_.translation() = cylinder_position;
                        cylinder_1_.dimensions.resize ( 1 );
                        cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                        cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                        test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                        test_node.removeCollisionObject(0);
                        test_node.removeCollisionObject(99 );
                        test_node.addCollisionObject ( octomap_, octo, 99 );
                        is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                        if (is_in_collision == 1) {
                            break;
                        }
                    }
                    /***********************************************************/
                    if (is_in_collision) {
                        cout << "4444444444444444444444444444444444444444" << endl;
                        continue;
                    }
                    else {
                        collision = 0; // 避开了障碍物，collision置零
                        break;
                    }
                }
            }
            else if (random_number == 3) {
                for (int i3=0; i3<1; i3++) { // 尝试50次随机旋转
                    rad = random_angle * pi / 180; // 转化为弧度值
                    Rz << cos(rad), -sin(rad), 0,   sin(rad), cos(rad), 0,   0, 0, 1;
                    p3 = Rz * p3;
                    /***********************************************************/
                    for (int iii1=0; iii1<(segmentation+1); iii1++) {
                        cylinder_position = p3 + iii1 * (p4 - p3) / segmentation;
                        cylinder_.linear() = pw_rotation;
                        cylinder_.translation() = cylinder_position;
                        cylinder_1_.dimensions.resize ( 1 );
                        cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                        cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                        test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                        test_node.removeCollisionObject(0);
                        test_node.removeCollisionObject(99 );
                        test_node.addCollisionObject ( octomap_, octo, 99 );
                        is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                        if (is_in_collision == 1) {
                            break;
                        }
                    }
                    /***********************************************************/
                    if (is_in_collision) {
                        cout << "55555555555555555555555555555555" << endl;
                        continue;
                    }
                    else {
                        collision = 0; // 避开了障碍物，collision置零
                        break;
                    }
                }
            }
        }
        random_number = 0;
        if (collision == 1) { // 如果还有碰撞，则重新做前向
            cout << "6666666666666666666666666666666666666666666" << endl;
            p4 = p4_0; // 将原先记录下来的p4_0赋给p4
            p3 = p3_0; // 将原先记录下来的p3_0赋给p3
            failedAttempts = failedAttempts + 1; 
            continue;
        }

        l = distanceCost3(p2, p3);
        cout << "222 l" << l << endl;
        p2_0 = p2; // 将原先的p2记录下来
        p2[0] = (l - d2) / l * p3[0] + d2 / l * p2[0];
        p2[1] = (l - d2) / l * p3[1] + d2 / l * p2[1];
        p2[2] = (l - d2) / l * p3[2] + d2 / l * p2[2];
        /***********************************************************/
        for (int iii1=0; iii1<(segmentation+1); iii1++) {
            cylinder_position = p2 + iii1 * (p3 - p2) / segmentation;
            cylinder_.linear() = pw_rotation;
            cylinder_.translation() = cylinder_position;
            cylinder_1_.dimensions.resize ( 1 );
            cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
            cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
            test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
            test_node.removeCollisionObject(0);
            test_node.removeCollisionObject(99 );
            test_node.addCollisionObject ( octomap_, octo, 99 );
            is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
            if (is_in_collision == 1) {
                break;
            }
        }
        /***********************************************************/

        if (is_in_collision) {
            cout << "777777777777777777777777777777777" << endl;
            collision = 1;
            // 有碰撞再生成随机数
            while (random_number == 0) {
                random_number = (rand() % (4-1)) + 1;
                cout << "222222222222222222222random_number：" << random_number << endl;
            }
            random_angle = (rand() % (361 - 1)) + 1;
            cout << "222222222222222222222random_angle：" << random_angle << endl;
        }
        else {
            collision = 0;
        }

        if (collision == 1) {
            cout << "88888888888888888888888888888888888" << endl;
            if (random_number == 1) {
                for (int j1=0; j1<1; j1++) { // 尝试50次随机旋转
                    rad = random_angle * pi / 180; // 转化为弧度值
                    Rx << 1, 0, 0,   0, cos(rad), -sin(rad),   0, sin(rad), cos(rad);
                    p2 = Rx * p2; 
                    /***********************************************************/
                    for (int iii1=0; iii1<(segmentation+1); iii1++) {
                        cylinder_position = p2 + iii1 * (p3 - p2) / segmentation;
                        cylinder_.linear() = pw_rotation;
                        cylinder_.translation() = cylinder_position;
                        cylinder_1_.dimensions.resize ( 1 );
                        cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                        cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                        test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                        test_node.removeCollisionObject(0);
                        test_node.removeCollisionObject(99 );
                        test_node.addCollisionObject ( octomap_, octo, 99 );
                        is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                        if (is_in_collision == 1) {
                            break;
                        }
                    }
                    /***********************************************************/
                    if (is_in_collision) {
                        cout << "999999999999999999999999999999" << endl;
                        continue;
                    }
                    else {
                        collision = 0; // 避开了障碍物，collision置零
                        break;
                    }
                }
            }
            else if (random_number == 2) {
                for (int j2=0; j2<1; j2++) { // 尝试50次随机旋转
                    rad = random_angle * pi / 180; // 转化为弧度值
                    Ry << cos(rad), 0, sin(rad),   0, 1, 0,   -sin(rad), 0, cos(rad);
                    p2 = Ry * p2;
                    /***********************************************************/
                    for (int iii1=0; iii1<(segmentation+1); iii1++) {
                        cylinder_position = p2 + iii1 * (p3 - p2) / segmentation;
                        cylinder_.linear() = pw_rotation;
                        cylinder_.translation() = cylinder_position;
                        cylinder_1_.dimensions.resize ( 1 );
                        cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                        cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                        test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                        test_node.removeCollisionObject(0);
                        test_node.removeCollisionObject(99 );
                        test_node.addCollisionObject ( octomap_, octo, 99 );
                        is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                        if (is_in_collision == 1) {
                            break;
                        }
                    }
                    /***********************************************************/
                    if (is_in_collision) {
                        cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaa" << endl;
                        continue;
                    }
                    else {
                        collision = 0; // 避开了障碍物，collision置零
                        break;
                    }
                }
            }
            else if (random_number == 3) {
                for (int j3=0; j3<1; j3++) { // 尝试50次随机旋转
                    rad = random_angle * pi / 180; // 转化为弧度值
                    Rz << cos(rad), -sin(rad), 0,   sin(rad), cos(rad), 0,   0, 0, 1;
                    p2 = Rz * p2;
                    /***********************************************************/
                    for (int iii1=0; iii1<(segmentation+1); iii1++) {
                        cylinder_position = p2 + iii1 * (p3 - p2) / segmentation;
                        cylinder_.linear() = pw_rotation;
                        cylinder_.translation() = cylinder_position;
                        cylinder_1_.dimensions.resize ( 1 );
                        cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                        cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                        test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                        test_node.removeCollisionObject(0);
                        test_node.removeCollisionObject(99 );
                        test_node.addCollisionObject ( octomap_, octo, 99 );
                        is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                        if (is_in_collision == 1) {
                            break;
                        }
                    }
                    /***********************************************************/
                    if (is_in_collision) {
                        cout << "bbbbbbbbbbbbbbbbbbbbbbbbb" << endl;
                        continue;
                    }
                    else {
                        collision = 0; // 避开了障碍物，collision置零
                        break;
                    }
                }
            }
        }
        random_number = 0;
        if (collision == 1) { // 如果还有碰撞，则重新做前向
            cout << "cccccccccccccccccccccccccccccccccccccccccccc" << endl;
            p4 = p4_0; // 将原先记录下来的p4_0赋给p4
            p3 = p3_0; // 将原先记录下来的p3_0赋给p3
            p2 = p2_0; // 将原先记录下来的p2_0赋给p2
            failedAttempts = failedAttempts + 1;
            continue;
        }

        l = distanceCost3(p1, p2);
        cout << "333 l" << l << endl;
        p1_0 = p1; // 将原先的p1记录下来
        p1[0] = (l - d1) / l * p2[0] + d1 / l * p1[0];
        p1[1] = (l - d1) / l * p2[1] + d1 / l * p1[1];
        p1[2] = (l - d1) / l * p2[2] + d1 / l * p1[2];
        /***********************************************************/
        for (int iii1=0; iii1<(segmentation+1); iii1++) {
            cylinder_position = p1 + iii1 * (p2 - p1) / segmentation;
            cylinder_.linear() = pw_rotation;
            cylinder_.translation() = cylinder_position;
            cylinder_1_.dimensions.resize ( 1 );
            cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
            cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
            test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
            test_node.removeCollisionObject(0);
            test_node.removeCollisionObject(99 );
            test_node.addCollisionObject ( octomap_, octo, 99 );
            is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
            if (is_in_collision == 1) {
                break;
            }
        }
        /***********************************************************/
        if (is_in_collision) {
            cout << "dddddddddddddddddddddddddddd" << endl;
            collision = 1;
            while (random_number == 0) {
                random_number = (rand() % (4-1)) + 1;
                cout << "3333333333333333333333333random_number：" << random_number << endl;
            }
            random_angle = (rand() % (361 - 1)) + 1; // 随机旋转角度
            cout << "3333333333333333333333333random_angle：" << random_angle << endl;
        }
        else {
            collision = 0;
        }
        if (collision == 1) {
            cout << "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee" << endl;
            if (random_number == 1) {
                for (int k1=0; k1<1; k1++) { // 尝试50次随机旋转
                    rad = random_angle * pi / 180; // 转化为弧度值
                    Rx << 1, 0, 0,   0, cos(rad), -sin(rad),   0, sin(rad), cos(rad);
                    p1 = Rx * p1;
                    /***********************************************************/
                    for (int iii1=0; iii1<(segmentation+1); iii1++) {
                        cylinder_position = p1 + iii1 * (p2 - p1) / segmentation;
                        cylinder_.linear() = pw_rotation;
                        cylinder_.translation() = cylinder_position;
                        cylinder_1_.dimensions.resize ( 1 );
                        cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                        cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                        test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                        test_node.removeCollisionObject(0);
                        test_node.removeCollisionObject(99 );
                        test_node.addCollisionObject ( octomap_, octo, 99 );
                        is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                        if (is_in_collision == 1) {
                            break;
                        }
                    }
                    /***********************************************************/
                    if (is_in_collision) {
                        cout << "ffffffffffffffffffffffffffffffffffffffffffffffffffff" << endl;
                        continue;
                    }
                    else {
                        collision = 0; // 避开了障碍物，collision置零
                        break;
                    }
                }
            }
            else if (random_number == 2) {
                for (int k2=0; k2<1; k2++) { // 尝试50次随机旋转
                    rad = random_angle * pi / 180; // 转化为弧度值
                    Ry << cos(rad), 0, sin(rad),   0, 1, 0,   -sin(rad), 0, cos(rad);
                    p1 = Ry * p1;
                    /***********************************************************/
                    for (int iii1=0; iii1<(segmentation+1); iii1++) {
                        cylinder_position = p1 + iii1 * (p2 - p1) / segmentation;
                        cylinder_.linear() = pw_rotation;
                        cylinder_.translation() = cylinder_position;
                        cylinder_1_.dimensions.resize ( 1 );
                        cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                        cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                        test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                        test_node.removeCollisionObject(0);
                        test_node.removeCollisionObject(99 );
                        test_node.addCollisionObject ( octomap_, octo, 99 );
                        is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                        if (is_in_collision == 1) {
                            break;
                        }
                    }
                    /***********************************************************/
                    if (is_in_collision){
                        cout << "gggggggggggggggggggggggggggg" << endl;
                        continue;
                    }
                    else {
                        collision = 0; // 避开了障碍物，collision置零
                        break;
                    }
                }
            }
            else if (random_number == 3) {
                for (int k3=0; k3<1; k3++) { // 尝试50次随机旋转
                    rad = random_angle * pi / 180; // 转化为弧度值
                    Rz << cos(rad), -sin(rad), 0,   sin(rad), cos(rad), 0,   0, 0, 1;
                    p1 = Rz * p1;
                    if (is_in_collision) {
                        cout << "hhhhhhhhhhhhhhhhhhhhhhhhh" << endl;
                        continue;
                    }
                    else {
                        collision = 0; // 避开了障碍物，collision置零
                        break;
                    }
                }
            }
        }
        random_number = 0;
        if (collision == 1) { // 如果还有碰撞，则重新做前向
            cout << "iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii" << endl;
            p4 = p4_0; // 将原先记录下来的p4_0赋给p4
            p3 = p3_0; // 将原先记录下来的p3_0赋给p3
            p2 = p2_0; // 将原先记录下来的p2_0赋给p2
            p1 = p1_0; // 将原先记录下来的p1_0赋给p1
            failedAttempts = failedAttempts + 1; 
            continue;
        }

        // // 施加关节限制
        // // 求p3p2与p3p4的夹角，施加角度限制（右肘部）
        // p3p2_direction_vector = p2 - p3;
        // p3p4_direction_vector = p4 - p3;
        // cosrad_1 = fabs(p3p2_direction_vector.dot(p3p4_direction_vector)) / (sqrt(p3p2_direction_vector.dot(p3p2_direction_vector)) * sqrt(p3p4_direction_vector.dot(p3p4_direction_vector))); // dot两向量取内积，abs取绝对值
        // rad_1 = acos(cosrad_1); // 所求角度只能为正，且∈[0,pi]
        // theta_1 = rad2deg(rad_1);
        // negative_theta_1 = -theta_1;
        // printf("前向theta_1 = %f\n", theta_1); 
        // printf("前向negative_theta_1 = %f\n", negative_theta_1);
        // if (negative_theta_1<-225 && theta_1>45) {
        //     printf("！！！！！！前向前向前向右肘部右肘部右肘部超出关节限制！！！！！！\n"); 
        //     p4 = p4_0; // 将原先记录下来的p4_0赋给p4
        //     p3 = p3_0; // 将原先记录下来的p3_0赋给p3
        //     p2 = p2_0; // 将原先记录下来的p2_0赋给p2
        //     p1 = p1_0; // 将原先记录下来的p1_0赋给p1
        //     failedAttempts = failedAttempts + 1; 
        //     continue;
        // }
        // // 求p2p1与p2p3之间的夹角，施加角度限制（侧抬）
        // p2p1_direction_vector = p1 - p2;
        // p2p3_direction_vector = p3 - p2;
        // cosrad_2 = fabs(p2p1_direction_vector.dot(p2p3_direction_vector)) / (sqrt(p2p1_direction_vector.dot(p2p1_direction_vector)) * sqrt(p2p3_direction_vector.dot(p2p3_direction_vector))); // dot两向量取内积，abs取绝对值
        // rad_2 = acos(cosrad_2);
        // theta_2 = rad2deg(rad_2);
        // negative_theta_2 = -theta_2;
        // printf("前向theta_2 = %f\n", theta_2);
        // printf("前向negative_theta_2 = %f\n", negative_theta_2);
        // if (negative_theta_2<-180 && theta_2>45) {
        //     printf("！！！！！！前向前向前向侧抬侧抬侧抬超出关节限制！！！！！！\n"); 
        //     p4 = p4_0;
        //     p3 = p3_0;
        //     p2 = p2_0;
        //     p1 = p1_0;
        //     failedAttempts = failedAttempts + 1; 
        //     continue;
        // }
        
        cout << "p1和p1_start的距离" << distanceCost3(p1, p1_start) << endl;
        if (distanceCost3(p1, p1_start) < forward_range) { // 如果前向FABRIK后p1在一定范围内，则退出循环
        // 腰部构型等后面再定
            // abc = calculate_pw_abc(p1(1),p1(2),p1(3), p2(1),p2(2),p2(3), pw(1),pw(2),pw(3));
            // waist_distance_min = distanceCost3(abc(1,:),pw);
            // for (i_waist = 1:length(abc(:,1))) {
            //     if (distanceCost3(abc(i_waist,:),pw) <= waist_distance_min) {
            //         waist_distance_min = distanceCost3(abc(i_waist,:),pw);
            //         pw_abc = abc(i_waist,:);
            //     }
            // }
            // printf("找到了最终构型\n");
            // failedAttempts = 0;
            
            /********************************************************** 计算腰部关节点的位置 ***************************************************/
            
            pw_xyz_ = calculate_pw_xyz(p1, p2, pw);
            double s = distanceCost3(pw_xyz_, p1);
            pw_xyz[0] = p1[0] - body_length / s * (p1[0] - pw_xyz_[0]);
            pw_xyz[1] = p1[1] - body_length / s * (p1[1] - pw_xyz_[1]);
            pw_xyz[2] = p1[2] - body_length / s * (p1[2] - pw_xyz_[2]);

            pw_Affine3d.linear() = pw_rotation;
            pw_Affine3d.translation() = pw_xyz;
            // body.linear() = body_rotation;
            // body.translation() = pw_xyz + (p1 - pw_xyz) / 2;

            // 打印各关节点以备验证使用
            cout << "p1：" << p1.transpose() << endl;
            cout << "p2：" << p2.transpose() << endl;
            cout << "p3：" << p3.transpose() << endl;
            cout << "p4：" << p4.transpose() << endl;
            cout << "pw_xyz：" << pw_xyz.transpose() << endl;
            cout << "pw：" << pw.transpose() << endl; // 0.25, -0.8, 1.0

            /********************************************************** 计算各关节角度 ***************************************************/
            // 计算移动平台前后左右位移
            // car_qianhou = pw_position[1] - pw_xyz[1];
            // car_zuoyou = pw_position[0] - pw_xyz[0];

            car_qianhou = pw_xyz[1] - pw[1]; // 前移是正
            car_zuoyou = pw_xyz[0] - pw[0]; // 右移是正

            // 计算腰部上下移动的距离和弯腰侧腰角度
            up_down_waist = pw_xyz[2] - pw[2]; // 腰部上下移动距离，上正下负
            if (up_down_waist > 0.1 || up_down_waist < -0.1) { // 上下10厘米，超过关节限制，重新来
                cout << "up_down_waist超过关节限制，重新来：" << up_down_waist << endl;
                sleep(10); // *******************************************
                p4 = p4_start_1; // 将原先记录下来的p4_0赋给p4
                p3 = p3_start_1; // 将原先记录下来的p3_0赋给p3
                p2 = p2_start_1; // 将原先记录下来的p2_0赋给p2
                p1 = p1_start_1; // 将原先记录下来的p1_0赋给p1
                failedAttempts = failedAttempts + 1; 
                continue;
            }
            // double waist_zuoyou = pw_xyz[0] - pw[0]; % 腰部左右移动距离，右正左负
            // double waist_qianhou = pw_xyz(2)-pw(2); % 腰部前后移动距离，前正后负
            /********************************* 脚部旋转角度 **************************************/
            // 计算
            rad_jiaobu = calculate_jiaobu(p1, p2);
            // 判断正负
            if (p2[1] - p2_start_1[1]<0) { // 脚部向右旋转，角度为正
                rad_jiaobu = -rad_jiaobu;
            }
            theta_jiaobu = rad2deg(rad_jiaobu);
            /********************************* 弯腰侧腰角度 **************************************/
            // 计算
            Eigen::Vector2d rad_wanyao_ceyao = calculate_wanyao_ceyao(pw_xyz, p1, rad_jiaobu);
            rad_wanyao = rad_wanyao_ceyao[0]; // 这里theta均为正，但具体是正还是负，看绕旋转轴顺时针还是逆时针旋转
            rad_ceyao = rad_wanyao_ceyao[1];
            theta_wanyao = rad2deg(rad_wanyao);
            theta_ceyao = rad2deg(rad_ceyao);
            // 判断弯腰侧腰的角度正负
            if (p1[1]-pw_xyz[1]>0) { // 向前弯腰，角度为负
                rad_wanyao = -rad_wanyao;
                theta_wanyao = -theta_wanyao;
            }
            if (p1[0]-pw_xyz[0]>0) { // 向右侧腰，角度为负
                rad_ceyao = -rad_ceyao;
                theta_ceyao = -theta_ceyao;
            }
            if (theta_wanyao > 20 || theta_wanyao < -20) { // 弯腰[-45,45]，超过关节限制，重新来
                cout << "theta_wanyao超过关节限制，重新来：" << theta_wanyao << endl;
                sleep(10); // *******************************************
                p4 = p4_start_1; // 将原先记录下来的p4_0赋给p4
                p3 = p3_start_1; // 将原先记录下来的p3_0赋给p3
                p2 = p2_start_1; // 将原先记录下来的p2_0赋给p2
                p1 = p1_start_1; // 将原先记录下来的p1_0赋给p1
                failedAttempts = failedAttempts + 1; 
                continue;
            }
            if (theta_ceyao > 20 || theta_ceyao < -20) { // 侧腰[-45,45]，超过关节限制，重新来
                cout << "theta_ceyao超过关节限制，重新来：" << theta_ceyao << endl;
                sleep(10); // *******************************************
                p4 = p4_start_1; // 将原先记录下来的p4_0赋给p4
                p3 = p3_start_1; // 将原先记录下来的p3_0赋给p3
                p2 = p2_start_1; // 将原先记录下来的p2_0赋给p2
                p1 = p1_start_1; // 将原先记录下来的p1_0赋给p1
                failedAttempts = failedAttempts + 1; 
                continue;
            }
            /********************************* 侧抬、前后抬角度 **************************************/
            // 计算
            Eigen::Vector2d rad_qianhoutai_cetai = calculate_qianhoutai_cetai(pw_xyz, p1, p2, p3);
            rad_qianhoutai = rad_qianhoutai_cetai[0]; // 这里theta均为正，但具体是正还是负，看绕旋转轴顺时针还是逆时针旋转
            rad_cetai = rad_qianhoutai_cetai[1];
            // rad_o = rad_qianhoutai_cetai[2];
            theta_qianhoutai = rad2deg(rad_qianhoutai); // 这里theta均为正，但具体是正还是负，看绕旋转轴顺时针还是逆时针旋转
            theta_cetai = rad2deg(rad_cetai);
            // theta_o = rad2deg(rad_jiaobu);
            /********************************* 判断侧抬、前后抬正负 **************************************/
            vector<double> theta_qianhoutai_cetai_vertify = calculate_qianhoutai_cetai_vertify(p3, pw_xyz, 
                                                                                                                        theta_wanyao, theta_ceyao, theta_jiaobu, theta_qianhoutai, theta_cetai);
            theta_qianhoutai = theta_qianhoutai_cetai_vertify[0];
            theta_cetai = theta_qianhoutai_cetai_vertify[1];
            if (theta_cetai > 45 || theta_cetai < -225) { // 侧抬[-225,45]，超过关节限制，重新来
                cout << "theta_cetai超过关节限制，重新来：" << theta_cetai << endl;
                sleep(10); // *******************************************
                p4 = p4_start_1; // 将原先记录下来的p4_0赋给p4
                p3 = p3_start_1; // 将原先记录下来的p3_0赋给p3
                p2 = p2_start_1; // 将原先记录下来的p2_0赋给p2
                p1 = p1_start_1; // 将原先记录下来的p1_0赋给p1
                failedAttempts = failedAttempts + 1; 
                continue;
            }


            // 计算大臂旋转、肘部旋转角度
            rad_wanyao = deg2rad(theta_wanyao);
            double c6 = cos(rad_wanyao); double s6 = sin(rad_wanyao);
            rad_ceyao = deg2rad(theta_ceyao);
            double c7 = cos(rad_ceyao); double s7 = sin(rad_ceyao);
            rad_qianhoutai = deg2rad(theta_qianhoutai); // 更新rad_qianhoutai
            rad_cetai = deg2rad(theta_cetai); // 更新rad_cetai
            double c8 = cos(rad_qianhoutai); double s8 = sin(rad_qianhoutai);
            rad_jiaobu = deg2rad(theta_jiaobu);
            double co = cos(rad_jiaobu); double so = sin(rad_jiaobu);
            cout << "c6：" << c6 << endl; cout << "s6：" << s6 << endl;
            cout << "c7：" << c7 << endl; cout << "s7：" << s7 << endl;
            cout << "c8：" << c8 << endl; cout << "s8：" << s8 << endl;
            cout << "co：" << co << endl; cout << "so：" << so << endl;
            double c9 = cos(rad_cetai); double s9 = sin(rad_cetai); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!注意要调整rad_cetai正负，这对后面求的角度有影响！
            cout << "c9：" << c9 << endl; cout << "s9：" << s9 << endl;
            Eigen::Matrix4d T9G = calculate_T9G(pw_xyz, c6, s6, c7, s7, co, so, c8, s8 ,c9, s9);
            cout << "T9G：" << T9G << endl;
            Eigen::Vector4d p49_pre;
            p49_pre << 0, big_arm_length, small_arm_length, 1; // p4相对于9坐标系的坐标
            Eigen::Vector4d p4G_pre = T9G * p49_pre;
            Eigen::Vector4d pp = p4G_pre;
            cout << "pp：" << pp.transpose() << endl;
            Eigen::Vector2d rad_dabi_zhoubu = calculate_dabi_zhoubu(p2, p3, p4, pp);
            rad_dabi = rad_dabi_zhoubu[0]; // 这里theta均为正，但具体是正还是负，看绕旋转轴顺时针还是逆时针旋转
            rad_zhoubu = rad_dabi_zhoubu[1];
            theta_dabi = rad2deg(rad_dabi);
            theta_zhoubu = rad2deg(rad_zhoubu);
            /********************************* 判断大臂旋转、肘部旋转的角度正负 **************************************/
            vector<double> theta_dabi_zhoubu_vertify = calculate_dabi_zhoubu_vertify(p_goal, pw_xyz, 
                                                                                                                        theta_wanyao, theta_ceyao, theta_jiaobu, theta_qianhoutai, theta_cetai, 
                                                                                                                        theta_dabi, theta_zhoubu);
            theta_dabi = theta_dabi_zhoubu_vertify[0];
            theta_zhoubu = theta_dabi_zhoubu_vertify[1];
            // 根据theta更新大臂、肘部的rad
            rad_dabi = deg2rad(theta_dabi);
            rad_zhoubu = deg2rad(theta_zhoubu);
            cout << "88888888888888888888888888：" << theta_zhoubu << endl;
            if (theta_zhoubu > 45 || theta_zhoubu < -180) { // 肘部[-90,45]，超过关节限制，重新来
                cout << "theta_zhoubu超过关节限制，重新来："<< theta_zhoubu << endl;
                sleep(10); // *******************************************
                p4 = p4_start_1; // 将原先记录下来的p4_0赋给p4
                p3 = p3_start_1; // 将原先记录下来的p3_0赋给p3
                p2 = p2_start_1; // 将原先记录下来的p2_0赋给p2
                p1 = p1_start_1; // 将原先记录下来的p1_0赋给p1
                failedAttempts = failedAttempts + 1; 
                continue;
            }
            /******************************************************************************************************************************/
            // 若均在关节限度内，则退出
            printf("从forward_range退出，找到了最终构型\n");
            break;
        }
        flag_backward = 1;

        /***************************************** 后向FABRIK ******************************************/
        while (flag_backward == 1) {
            p1_1 = p1; // 将前向过后的p1记录下来
            p1 = p1_start; // 把p1拉到p1_start
            l = distanceCost3(p1, p2);
            cout << "444 l" << l << endl;
            p2_1 = p2; // 将前向过后的p2记录下来
            p2[0] = (l - d1) / l * p1[0] + d1 / l * p2[0];
            p2[1] = (l - d1) / l * p1[1] + d1 / l * p2[1];
            p2[2] = (l - d1) / l * p1[2] + d1 / l * p2[2];
            /***********************************************************/
            for (int iii1=0; iii1<(segmentation+1); iii1++) {
                cylinder_position = p1 + iii1 * (p2 - p1) / segmentation;
                cylinder_.linear() = pw_rotation;
                cylinder_.translation() = cylinder_position;
                cylinder_1_.dimensions.resize ( 1 );
                cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                test_node.removeCollisionObject(0);
                test_node.removeCollisionObject(99 );
                test_node.addCollisionObject ( octomap_, octo, 99 );
                is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                if (is_in_collision == 1) {
                    break;
                }
            }
            /***********************************************************/
            if (is_in_collision) {
                cout << "jjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj" << endl;
                collision = 1;
                while (random_number == 0) {
                    random_number = (rand() % (4-1)) + 1;
                    cout << "44444444444444444random_number：" << random_number << endl;
                }
                random_angle = (rand() % (361 - 1)) + 1; // 随机旋转角度
                cout << "44444444444444444random_angle：" << random_angle << endl;
            }
            else {
                collision = 0;
            }
            if (collision == 1) {
                cout << "kkkkkkkkkkkkkkkkkkkkkkkkkkkkkk" << endl;
                if (random_number == 1) {
                    for (int ii1=0; ii1<1; ii1++) { // 尝试50次随机旋转
                        rad = random_angle * pi / 180; // 转化为弧度值
                        Rx << 1, 0, 0,   0, cos(rad), -sin(rad),   0, sin(rad), cos(rad);
                        p2 = Rx * p2;
                        /***********************************************************/
                        for (int iii1=0; iii1<(segmentation+1); iii1++) {
                            cylinder_position = p1 + iii1 * (p2 - p1) / segmentation;
                            cylinder_.linear() = pw_rotation;
                            cylinder_.translation() = cylinder_position;
                            cylinder_1_.dimensions.resize ( 1 );
                            cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                            cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                            test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                            test_node.removeCollisionObject(0);
                            test_node.removeCollisionObject(99 );
                            test_node.addCollisionObject ( octomap_, octo, 99 );
                            is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                            if (is_in_collision == 1) {
                                break;
                            }
                        }
                        /***********************************************************/
                        if (is_in_collision) {
                            cout << "lllllllllllllllllllllllllllllllllllllll" << endl;
                            continue;
                        }
                        else {
                            collision = 0; // 避开了障碍物，collision置零
                            break;
                        }
                    }
                }
                else if (random_number == 2) {
                    for (int ii2=0; ii2<1; ii2++) { // 尝试50次随机旋转
                        rad = random_angle * pi / 180; // 转化为弧度值
                        Ry << cos(rad), 0, sin(rad),   0, 1, 0,   -sin(rad), 0, cos(rad);
                        p2 = Ry * p2;
                        /***********************************************************/
                        for (int iii1=0; iii1<(segmentation+1); iii1++) {
                            cylinder_position = p1 + iii1 * (p2 - p1) / segmentation;
                            cylinder_.linear() = pw_rotation;
                            cylinder_.translation() = cylinder_position;
                            cylinder_1_.dimensions.resize ( 1 );
                            cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                            cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                            test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                            test_node.removeCollisionObject(0);
                            test_node.removeCollisionObject(99 );
                            test_node.addCollisionObject ( octomap_, octo, 99 );
                            is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                            if (is_in_collision == 1) {
                                break;
                            }
                        }
                        /***********************************************************/
                        if (is_in_collision) {
                            cout << "mmmmmmmmmmmmmmmmmmm" << endl;
                            continue;
                        }
                        else {
                            collision = 0; // 避开了障碍物，collision置零
                            break;
                        }
                    }
                }
                else if (random_number == 3) {
                    for (int ii3=0; ii3<1; ii3++) { // 尝试50次随机旋转
                        rad = random_angle * pi / 180; // 转化为弧度值
                        Rz << cos(rad), -sin(rad), 0,   sin(rad), cos(rad), 0,   0, 0, 1;
                        p2 = Rz * p2;
                        /***********************************************************/
                        for (int iii1=0; iii1<(segmentation+1); iii1++) {
                            cylinder_position = p1 + iii1 * (p2 - p1) / segmentation;
                            cylinder_.linear() = pw_rotation;
                            cylinder_.translation() = cylinder_position;
                            cylinder_1_.dimensions.resize ( 1 );
                            cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                            cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                            test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                            test_node.removeCollisionObject(0);
                            test_node.removeCollisionObject(99 );
                            test_node.addCollisionObject ( octomap_, octo, 99 );
                            is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                            if (is_in_collision == 1) {
                                break;
                            }
                        }
                        /***********************************************************/
                        if (is_in_collision) {
                            cout << "nnnnnnnnnnnnnnnnnnnnnn" << endl;
                            continue;
                        }
                        else {
                            collision = 0; // 避开了障碍物，collision置零
                            break;
                        }
                    }
                }
            }
            random_number = 0;
            if (collision == 1) { // 如果还有碰撞，则重新做前向
                cout << "ooooooooooooooooooooooooooooo" << endl;
                p4 = p4_0; // 将原先记录下来的p4_0赋给p3
                p3 = p3_0; // 将原先记录下来的p3_0赋给p3
                p2 = p2_0; // 将原先记录下来的p2_0赋给p2
                p1 = p1_0; // 将原先记录下来的p1_0赋给p1
                failedAttempts = failedAttempts + 1; 
                continue;
            }

            l = distanceCost3(p2, p3);
            cout << "555 l" << l << endl;
            p3_1 = p3; // 将原先的p2记录下来
            p3[0] = (l - d2) / l * p2[0] + d2 / l * p3[0];
            p3[1] = (l - d2) / l * p2[1] + d2 / l * p3[1];
            p3[2] = (l - d2) / l * p2[2] + d2 / l * p3[2];
            /***********************************************************/
            for (int iii1=0; iii1<(segmentation+1); iii1++) {
                cylinder_position = p2 + iii1 * (p3 - p2) / segmentation;
                cylinder_.linear() = pw_rotation;
                cylinder_.translation() = cylinder_position;
                cylinder_1_.dimensions.resize ( 1 );
                cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                test_node.removeCollisionObject(0);
                test_node.removeCollisionObject(99 );
                test_node.addCollisionObject ( octomap_, octo, 99 );
                is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                if (is_in_collision == 1) {
                    break;
                }
            }
            /***********************************************************/
            if (is_in_collision) {
                cout << "ppppppppppppppppppppp" << endl;
                collision = 1;
                while (random_number == 0) {
                    random_number = (rand() % (4-1)) + 1;
                    cout << "5555555555555555555random_number：" << random_number << endl;
                }
                random_angle = (rand() % (361 - 1)) + 1; // 随机旋转角度
                cout << "5555555555555555555random_angle：" << random_angle << endl;
            }
            else {
                collision = 0;
            }
            if (collision == 1) {
                cout << "qqqqqqqqqqqqqqqqqqqqqqqqqqqqqq" << endl;
                if (random_number == 1) {
                    for (int jj1=0; jj1<1; jj1++) { // 尝试50次随机旋转
                        rad = random_angle * pi / 180; // 转化为弧度值
                        Rx << 1, 0, 0,   0, cos(rad), -sin(rad),   0, sin(rad), cos(rad);
                        p3 = Rx * p3;
                        /***********************************************************/
                        for (int iii1=0; iii1<(segmentation+1); iii1++) {
                            cylinder_position = p2 + iii1 * (p3 - p2) / segmentation;
                            cylinder_.linear() = pw_rotation;
                            cylinder_.translation() = cylinder_position;
                            cylinder_1_.dimensions.resize ( 1 );
                            cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                            cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                            test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                            test_node.removeCollisionObject(0);
                            test_node.removeCollisionObject(99 );
                            test_node.addCollisionObject ( octomap_, octo, 99 );
                            is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                            if (is_in_collision == 1) {
                                break;
                            }
                        }
                        /***********************************************************/
                        if (is_in_collision) {
                            cout << "rrrrrrrrrrrrrrrrrrrrrrrrrrrrr" << endl;
                            continue;
                        }
                        else {
                            collision = 0; // 避开了障碍物，collision置零
                            break;
                        }
                    }
                }
                else if (random_number == 2) {
                    for (int jj2=0; jj2<1; jj2++) { // 尝试50次随机旋转
                        rad = random_angle * pi / 180; // 转化为弧度值
                        Ry << cos(rad), 0, sin(rad),   0, 1, 0,   -sin(rad), 0, cos(rad);
                        p3 = Ry * p3;
                        /***********************************************************/
                        for (int iii1=0; iii1<(segmentation+1); iii1++) {
                            cylinder_position = p2 + iii1 * (p3 - p2) / segmentation;
                            cylinder_.linear() = pw_rotation;
                            cylinder_.translation() = cylinder_position;
                            cylinder_1_.dimensions.resize ( 1 );
                            cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                            cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                            test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                            test_node.removeCollisionObject(0);
                            test_node.removeCollisionObject(99 );
                            test_node.addCollisionObject ( octomap_, octo, 99 );
                            is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                            if (is_in_collision == 1) {
                                break;
                            }
                        }
                        /***********************************************************/
                        if (is_in_collision) {
                            cout << "ssssssssssssssssssssssssssssss" << endl;
                            continue;
                        }
                        else {
                            collision = 0; // 避开了障碍物，collision置零
                            break;
                        }
                    }
                }
                else if (random_number == 3) {
                    for (int jj3=0; jj3<1; jj3++) { // 尝试50次随机旋转
                        rad = random_angle * pi / 180; // 转化为弧度值
                        Rz << cos(rad), -sin(rad), 0,   sin(rad), cos(rad), 0,   0, 0, 1;
                        p3 = Rz * p3;
                        /***********************************************************/
                        for (int iii1=0; iii1<(segmentation+1); iii1++) {
                            cylinder_position = p2 + iii1 * (p3 - p2) / segmentation;
                            cylinder_.linear() = pw_rotation;
                            cylinder_.translation() = cylinder_position;
                            cylinder_1_.dimensions.resize ( 1 );
                            cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                            cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                            test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                            test_node.removeCollisionObject(0);
                            test_node.removeCollisionObject(99 );
                            test_node.addCollisionObject ( octomap_, octo, 99 );
                            is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                            if (is_in_collision == 1) {
                                break;
                            }
                        }
                        /***********************************************************/
                        if (is_in_collision) {
                            cout << "ttttttttttttttttttttttttttttttttttttttttt" << endl;
                            continue;
                        }
                        else {
                            collision = 0; // 避开了障碍物，collision置零
                            break;
                        }
                    }
                }
            }
            random_number = 0;
            if (collision == 1) { // 如果还有碰撞，则重新做前向
                cout << "uuuuuuuuuuuuuuuuuuuuuuuuu" << endl;
                p4 = p4_0; // 将原先记录下来的p4_0赋给p3
                p3 = p3_0; // 将原先记录下来的p3_0赋给p3
                p2 = p2_0; // 将原先记录下来的p2_0赋给p2
                p1 = p1_0; // 将原先记录下来的p1_0赋给p1
                failedAttempts = failedAttempts + 1;
                continue;
            }

            l = distanceCost3(p3, p4);
            cout << "666 l" << l << endl;
            p4_1 = p4; // 将原先的p1记录下来
            p4[0] = (l - d3) / l * p3[0] + d3 / l * p4[0];
            p4[1] = (l - d3) / l * p3[1] + d3 / l * p4[1];
            p4[2] = (l - d3) / l * p3[2] + d3 / l * p4[2];
            /***********************************************************/
            for (int iii1=0; iii1<(segmentation+1); iii1++) {
                cylinder_position = p3 + iii1 * (p4 - p3) / segmentation;
                cylinder_.linear() = pw_rotation;
                cylinder_.translation() = cylinder_position;
                cylinder_1_.dimensions.resize ( 1 );
                cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                test_node.removeCollisionObject(0);
                test_node.removeCollisionObject(99 );
                test_node.addCollisionObject ( octomap_, octo, 99 );
                is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                if (is_in_collision == 1) {
                    break;
                }
            }
            /***********************************************************/
            if (is_in_collision) {
                cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv" << endl;
                collision = 1;
                while (random_number == 0) {
                    random_number = (rand() % (4-1)) + 1;
                    cout << "6666666666666666666random_number" << random_number << endl;
                }
                random_angle = (rand() % (361 - 1)) + 1; // 随机旋转角度
                cout << "6666666666666666666random_angle" << random_angle << endl;
            }
            else {
                collision = 0;
            }
            if (collision == 1) {
                if (random_number == 1) {
                    for (int kk1=0; kk1<1; kk1++) { // 尝试50次随机旋转
                        rad = random_angle * pi / 180; // 转化为弧度值
                        Rx << 1, 0, 0,   0, cos(rad), -sin(rad),   0, sin(rad), cos(rad);
                        p4 = Rx * p4;
                        /***********************************************************/
                        for (int iii1=0; iii1<(segmentation+1); iii1++) {
                            cylinder_position = p3 + iii1 * (p4 - p3) / segmentation;
                            cylinder_.linear() = pw_rotation;
                            cylinder_.translation() = cylinder_position;
                            cylinder_1_.dimensions.resize ( 1 );
                            cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                            cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                            test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                            test_node.removeCollisionObject(0);
                            test_node.removeCollisionObject(99 );
                            test_node.addCollisionObject ( octomap_, octo, 99 );
                            is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                            if (is_in_collision == 1) {
                                break;
                            }
                        }
                        /***********************************************************/
                        if (is_in_collision) {
                            cout << "wwwwwwwwwwwwwwwwwwwwwww" << endl;
                            continue;
                        }
                        else {
                            collision = 0; // 避开了障碍物，collision置零
                            break;
                        }
                    }
                }
                else if (random_number == 2) {
                    for (int kk2=0; kk2<1; kk2++) { // 尝试50次随机旋转
                        rad = random_angle * pi / 180; // 转化为弧度值
                        Ry << cos(rad), 0, sin(rad),   0, 1, 0,   -sin(rad), 0, cos(rad);
                        p4 = Ry * p4;
                        /***********************************************************/
                        for (int iii1=0; iii1<(segmentation+1); iii1++) {
                            cylinder_position = p3 + iii1 * (p4 - p3) / segmentation;
                            cylinder_.linear() = pw_rotation;
                            cylinder_.translation() = cylinder_position;
                            cylinder_1_.dimensions.resize ( 1 );
                            cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                            cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                            test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                            test_node.removeCollisionObject(0);
                            test_node.removeCollisionObject(99 );
                            test_node.addCollisionObject ( octomap_, octo, 99 );
                            is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                            if (is_in_collision == 1) {
                                break;
                            }
                        }
                        /***********************************************************/
                        if (is_in_collision) {
                            cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << endl;
                            continue;
                        }
                        else {
                            collision = 0; // 避开了障碍物，collision置零
                            break;
                        }
                    }
                }
                else if (random_number == 3) {
                    for (int kk3=0; kk3<1; kk3++) { // 尝试50次随机旋转
                        rad = random_angle * pi / 180; // 转化为弧度值
                        Rz << cos(rad), -sin(rad), 0,   sin(rad), cos(rad), 0,   0, 0, 1;
                        p4 = Rz * p4;
                        /***********************************************************/
                        for (int iii1=0; iii1<(segmentation+1); iii1++) {
                            cylinder_position = p3 + iii1 * (p4 - p3) / segmentation;
                            cylinder_.linear() = pw_rotation;
                            cylinder_.translation() = cylinder_position;
                            cylinder_1_.dimensions.resize ( 1 );
                            cylinder_1_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
                            cylinder_1_.type=shape_msgs::SolidPrimitive::SPHERE;
                            test_node.addCollisionObject ( cylinder_1_, cylinder_, 0 );
                            test_node.removeCollisionObject(0);
                            test_node.removeCollisionObject(99 );
                            test_node.addCollisionObject ( octomap_, octo, 99 );
                            is_in_collision = test_node.checkCollisionObjectWorld(cylinder_1_, cylinder_); // 碰撞检测，若碰撞则返回true
                            if (is_in_collision == 1) {
                                break;
                            }
                        }
                        /***********************************************************/
                        if (is_in_collision) {
                            cout << "yyyyyyyyyyyyyyyyyyyyyyyyyyyyyy" << endl;
                            continue;
                        }
                        else {
                            collision = 0; // 避开了障碍物，collision置零
                            break;
                        }
                    }
                }
            }
            random_number = 0;
            if (collision == 1) { // 如果还有碰撞，则重新做前向
                cout << "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz" << endl;
                p4 = p4_0; // 将原先记录下来的p4_0赋给p3
                p3 = p3_0; // 将原先记录下来的p3_0赋给p3
                p2 = p2_0; // 将原先记录下来的p2_0赋给p2
                p1 = p1_0; // 将原先记录下来的p1_0赋给p1
                failedAttempts = failedAttempts + 1; 
                continue;
            }

            // // 施加关节限制
            // // 求p3p2与p3p4的夹角，施加角度限制（右肘部）
            // p3p2_direction_vector = p2 - p3;
            // p3p4_direction_vector = p4 - p3;
            // cosrad_1 = fabs(p3p2_direction_vector.dot(p3p4_direction_vector)) / (sqrt(p3p2_direction_vector.dot(p3p2_direction_vector)) * sqrt(p3p4_direction_vector.dot(p3p4_direction_vector))); // dot两向量取内积，abs取绝对值
            // rad_1 = acos(cosrad_1);
            // theta_1 = rad2deg(rad_1);
            // negative_theta_1 = -theta_1;
            // printf("后向theta_1 = %f\n", theta_1); 
            // printf("后向negative_theta_1 = %f\n", negative_theta_1);
            // if (negative_theta_1<-225 && theta_1>45) { // 所求角度只能为正
            //     printf("！！！！！！后向后向后向右肘部右肘部右肘部超出关节限制！！！！！！\n"); 
            //     p4 = p4_1; // 将原先记录下来的p4_0赋给p4
            //     p3 = p3_1; // 将原先记录下来的p3_0赋给p3
            //     p2 = p2_1; // 将原先记录下来的p2_0赋给p2
            //     p1 = p1_1; // 将原先记录下来的p1_0赋给p1
            //     failedAttempts = failedAttempts + 1; 
            //     continue;
            // }
            // // 求p2p1与p2p3之间的夹角，施加角度限制（侧抬）
            // p2p1_direction_vector = p1 - p2;
            // p2p3_direction_vector = p3 - p2;
            // cosrad_2 = fabs(p2p1_direction_vector.dot(p2p3_direction_vector)) / (sqrt(p2p1_direction_vector.dot(p2p1_direction_vector)) * sqrt(p2p3_direction_vector.dot(p2p3_direction_vector))); // dot两向量取内积，abs取绝对值
            // rad_2 = acos(cosrad_2);
            // theta_2 = rad2deg(rad_2);
            // negative_theta_2 = -theta_2;
            // printf("后向theta_2 = %f\n", theta_2);
            // printf("后向negative_theta_2 = %f\n", negative_theta_2);
            // if (negative_theta_2<-180 && theta_2>45) {
            //     printf("！！！！！！后向后向后向侧抬侧抬侧抬超出关节限制！！！！！！\n"); 
            //     p4 = p4_1; // 将原先记录下来的p4_0赋给p4
            //     p3 = p3_1; // 将原先记录下来的p3_0赋给p3
            //     p2 = p2_1; // 将原先记录下来的p2_0赋给p2
            //     p1 = p1_1; // 将原先记录下来的p1_0赋给p1
            //     failedAttempts = failedAttempts + 1; 
            //     continue;
            // }
            failedAttempts = failedAttempts + 1; 
            flag_backward = 0;
        }
        
        if (distanceCost3(p4, p_goal) < backward_range) { // 如果后向FABRIK后p1在一定范围内，则退出循环
            // abc = calculate_pw_abc(p1(1),p1(2),p1(3), p2(1),p2(2),p2(3), pw(1),pw(2),pw(3));
            // waist_distance_min = distanceCost3(abc(1,:),pw);
            // for (i_waist = 1:length(abc(:,1))) {
            //     if (distanceCost3(abc(i_waist,:),pw) <= waist_distance_min) {
            //         waist_distance_min = distanceCost3(abc(i_waist,:),pw);
            //         pw_abc = abc(i_waist,:);
            //     }
            // }
            // printf("找到了最终构型\n");
            // failedAttempts = 0;
            printf("从backward_range退出，找到了最终构型\n");
            break;
        }
    }    
    endTime = clock(); // 计时结束
    cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    // 输出各关节角度
    cout << "脚部旋转弧度：" << rad_jiaobu << endl;
    cout << "弯腰弧度：" << rad_wanyao << endl;
    cout << "侧腰弧度：" << rad_ceyao << endl;
    cout << "前后抬弧度：" << rad_qianhoutai << endl;
    cout << "侧抬弧度：" << rad_cetai << endl;
    cout << "大臂弧度：" << rad_dabi << endl;
    cout << "肘部弧度：" << rad_zhoubu << endl;

    cout << "car前后：" << car_qianhou << endl;
    cout << "car左右：" << car_zuoyou << endl;
    cout << "脚部旋转角度：" << theta_jiaobu << endl;
    cout << "上升下降高度：" << up_down_waist << endl;
    cout << "弯腰角度：" << theta_wanyao << endl;
    cout << "侧腰角度：" << theta_ceyao << endl;
    cout << "前后抬角度：" << theta_qianhoutai << endl;
    cout << "侧抬角度：" << theta_cetai << endl;
    cout << "大臂角度：" << theta_dabi << endl;
    cout << "肘部：" << theta_zhoubu << endl;

    // 弯腰、侧腰、up_down_waist要转化成L1、L2、L3上升下降的高度
    // 单位：m
    // 定义初始套筒高度L0，圆盘半径R，以及L1、L2、L3
    double L0 = waist_to_foot;
    double R = 0.09801; // 单位：m
    double L1, L2, L3;
    double root_3= pow(3,0.5);
    double cos_6 = cos(rad_wanyao), sin_6 = sin(rad_wanyao);
    double cos_7 = cos(rad_ceyao), sin_7 = sin(rad_ceyao);
    
    // 下面注释这部分theta6和theta7反了
    // L1 = -L0+sqrt(pow(0.5*R-0.5*R*cos_7-root_3*0.5*R*sin_6*sin_7,2)+pow (root_3*0.5*R-root_3*0.5*R*cos_6,2)+pow(-0.5*R*sin_7+root_3*0.5*R*sin_6*cos_7+L0+up_down_waist,2));
    // L2 = -L0+sqrt(pow(0.5*R-0.5*R*cos_7+root_3*0.5*R*sin_6*sin_7,2)+pow (-1*root_3*0.5*R+root_3*0.5*R*cos_6,2)+pow(-0.5*R*sin_7-root_3*0.5*R*sin_6*cos_7+L0+up_down_waist,2));
    // L3 = -L0+sqrt(pow(-R+R*cos_7,2)+pow(R*sin_7+L0+up_down_waist,2));
    L1 = -L0+sqrt(pow(-R+R*cos_6,2)+pow(R*sin_6+L0+up_down_waist,2));
    L2 = -L0+sqrt(pow(0.5*R-0.5*R*cos_6+root_3*0.5*R*sin_6*sin_7,2)+pow (-1*root_3*0.5*R+root_3*0.5*R*cos_7,2)+pow(-0.5*R*sin_6-root_3*0.5*R*sin_7*cos_6+L0+up_down_waist,2));
    L3 = -L0+sqrt(pow(0.5*R-0.5*R*cos_6-root_3*0.5*R*sin_6*sin_7,2)+pow (root_3*0.5*R-root_3*0.5*R*cos_7,2)+pow(-0.5*R*sin_6+root_3*0.5*R*sin_7*cos_6+L0+up_down_waist,2));
    cout << "L1 = " << L1 << endl;
    cout << "L2 = " << L2 << endl;
    cout << "L3 = " << L3 << endl;

    /************************************* 发布弧度信息，由fabrik_demo接收 *************************************/
    // ros::init(argc, argv, "send_rad_node");
    ros::Publisher rad_pub = nh.advertise<sensor_msgs::JointState>("send_rad", 100);
    sensor_msgs::JointState rad_value;
    // 车2+足部+上升下降（4）
    rad_value.name.push_back("car_qianhou");
    rad_value.name.push_back("car_zuoyou");
    // rad_value.name.push_back("car_z");
    rad_value.name.push_back("foot");
    rad_value.name.push_back("up_down_waist");
    // 腰部（2）
    rad_value.name.push_back("bend_waist");
    rad_value.name.push_back("side_waist");
    // 右臂（4）
    rad_value.name.push_back("qianhoutai");
    rad_value.name.push_back("cetai");
    rad_value.name.push_back("dabi");
    rad_value.name.push_back("zhoubu");
    // 腰部L1、L2、L3运动量
    rad_value.name.push_back("L1");
    rad_value.name.push_back("L2");
    rad_value.name.push_back("L3");
    // rad_value.name.push_back("xiaobi");
    rad_value.position.push_back(car_qianhou);
    rad_value.position.push_back(car_zuoyou);
    rad_value.position.push_back(rad_jiaobu);
    rad_value.position.push_back(up_down_waist);
    rad_value.position.push_back(rad_wanyao);
    rad_value.position.push_back(rad_ceyao);
    rad_value.position.push_back(rad_qianhoutai);
    rad_value.position.push_back(rad_cetai);
    rad_value.position.push_back(rad_dabi);
    rad_value.position.push_back(rad_zhoubu);
    rad_value.position.push_back(L1);
    rad_value.position.push_back(L2);
    rad_value.position.push_back(L3);
    
    // cout << "p1p2：" << distanceCost3(p1, p2) << endl;
    // cout << "p2p3：" << distanceCost3(p2, p3) << endl;
    // cout << "p3p4：" << distanceCost3(p3, p4) << endl;

    // 多发送几次
    ros::Rate loop_rate(10);
    while (ros::ok() && flag_finish < 30) {
        rad_pub.publish(rad_value);

        ros::spinOnce();
        loop_rate.sleep();
        flag_finish += 1;
    }
    // int i_send = 0;
    // while (i_send < 100) {
    //     i_send += 1;
    //     rad_pub.publish(rad_value);
    //     usleep(1000); // 延时1000us=1ms
    // }

    /**************************************************最终构型的模型显示（去掉）*********************************************/
    // octo_position<< 0.0, 0.0, 0.0; // octo
    // pw_position << 0.0, 0.0, 0.545; // pw
    // body_position << 0.0, 0.0, 0.6825; // 身体
    p1_position = p1; // p1
    // shoulder_position = half_p1_p2; // 肩部
    p2_position = p2; // p2
    // big_arm_position = half_p2_p3; // 大臂
    p3_position = p3; // p3
    // small_arm_position = half_p3_p4; // 小臂
    p4_position = p4; // p4

    // octo_rotation.setIdentity();
    // pw_rotation.setIdentity();
    // body_rotation.setIdentity();
    p1_rotation.setIdentity();
    // shoulder_rotation << 1, 0, 0,   0, 0, -1,   0, 1, 0; // 肩部绕x轴转90度
    p2_rotation.setIdentity();
    // big_arm_rotation.setIdentity();
    p3_rotation.setIdentity();
    // small_arm_rotation << 0, 0, 1,   0, 1, 0,   -1, 0, 0; // 小臂绕y轴转90度
    p4_rotation.setIdentity();

    // octo.linear() = octo_rotation;
    // octo.translation() = octo_position;
    // pw_Affine3d.linear() = pw_rotation;
    // pw_Affine3d.translation() = pw_position;
    // body.linear() = body_rotation;
    // body.translation() = body_position;
    p1_Affine3d.linear() = p1_rotation;
    p1_Affine3d.translation() = p1_position;
    // shoulder.linear() = shoulder_rotation;
    // shoulder.translation() = shoulder_position;
    p2_Affine3d.linear() = p2_rotation;
    p2_Affine3d.translation() = p2_position;
    // big_arm.linear() = big_arm_rotation;
    // big_arm.translation() = big_arm_position;
    p3_Affine3d.linear() = p3_rotation;
    p3_Affine3d.translation() = p3_position;
    // small_arm.linear() = small_arm_rotation;
    // small_arm.translation() = small_arm_position;
    p4_Affine3d.linear() = p4_rotation;
    p4_Affine3d.translation() = p4_position;

    // 绘制躯干
    // ros::Duration(5.0).sleep();    
    Eigen::Vector3d pw_xyzp1 = p1 - pw_xyz;
    // cout << pw_xyzp1.transpose() << endl;
    Eigen::Vector3d qugan_vec, qg1, qg2, qg3, qg4, qg5, qg6, qg7, qg8, qg9;
    Eigen::Affine3d qugan1, qugan2, qugan3, qugan4, qugan5, qugan6, qugan7, qugan8, qugan9;
    Eigen::Matrix3d sphere_rotation;
    sphere_rotation.setIdentity();
    shape_msgs::SolidPrimitive sphere1, sphere2, sphere3, sphere4, sphere5, sphere6, sphere7, sphere8, sphere9;

    sphere1.dimensions.resize ( 1 );
    sphere1.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere1.type=shape_msgs::SolidPrimitive::SPHERE;
    qugan_vec = pw_xyz + 0.1 * (p1 - pw_xyz);
    qg1 = qugan_vec;
    qugan1.linear() = sphere_rotation;
    qugan1.translation() = qg1;
    // cout << qugan_vec.transpose() << endl;
    
    sphere2.dimensions.resize ( 1 );
    sphere2.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere2.type=shape_msgs::SolidPrimitive::SPHERE;
    qugan_vec = pw_xyz + 0.2 * (p1 - pw_xyz);
    qg2 = qugan_vec;
    qugan2.linear() = sphere_rotation;
    qugan2.translation() = qg2;
    // cout << qugan_vec.transpose() << endl;
    
    sphere3.dimensions.resize ( 1 );
    sphere3.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere3.type=shape_msgs::SolidPrimitive::SPHERE;
    qugan_vec = pw_xyz +0.3 * (p1 - pw_xyz);
    qg3 = qugan_vec;
    qugan3.linear() = sphere_rotation;
    qugan3.translation() = qg3;
    // cout << qugan_vec.transpose() << endl;
    
    sphere4.dimensions.resize ( 1 );
    sphere4.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere4.type=shape_msgs::SolidPrimitive::SPHERE;
    qugan_vec = pw_xyz + 0.4 * (p1 - pw_xyz);
    qg4 = qugan_vec;
    qugan4.linear() = sphere_rotation;
    qugan4.translation() = qg4;
    // cout << qugan_vec.transpose() << endl;
    
    sphere5.dimensions.resize ( 1 );
    sphere5.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere5.type=shape_msgs::SolidPrimitive::SPHERE;
    qugan_vec = pw_xyz + 0.5 * (p1 - pw_xyz);
    qg5 = qugan_vec;
    qugan5.linear() = sphere_rotation;
    qugan5.translation() = qg5;
    // cout << qugan_vec.transpose() << endl;
    
    sphere6.dimensions.resize ( 1 );
    sphere6.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere6.type=shape_msgs::SolidPrimitive::SPHERE;
    qugan_vec = pw_xyz + 0.6 * (p1 - pw_xyz);
    qg6 = qugan_vec;
    qugan6.linear() = sphere_rotation;
    qugan6.translation() = qg6;
    // cout << qugan_vec.transpose() << endl;
    
    sphere7.dimensions.resize ( 1 );
    sphere7.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere7.type=shape_msgs::SolidPrimitive::SPHERE;
    qugan_vec = pw_xyz + 0.7 * (p1 - pw_xyz);
    qg7 = qugan_vec;
    qugan7.linear() = sphere_rotation;
    qugan7.translation() = qg7;
    // cout << qugan_vec.transpose() << endl;
    
    sphere8.dimensions.resize ( 1 );
    sphere8.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere8.type=shape_msgs::SolidPrimitive::SPHERE;
    qugan_vec = pw_xyz + 0.8 * (p1 - pw_xyz);
    qg8 = qugan_vec;
    qugan8.linear() = sphere_rotation;
    qugan8.translation() = qg8;
    // cout << qugan_vec.transpose() << endl;
    
    sphere9.dimensions.resize ( 1 );
    sphere9.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere9.type=shape_msgs::SolidPrimitive::SPHERE;
    qugan_vec = pw_xyz + 0.9 * (p1 - pw_xyz);
    qg9 = qugan_vec;
    qugan9.linear() = sphere_rotation;
    qugan9.translation() = qg9;
    // cout << qugan_vec.transpose() << endl;
    

    // // 绘制肩膀
    Eigen::Vector3d p1p2 = p2 - p1;
    Eigen::Vector3d jb_vec;
    Eigen::Affine3d jianbang1, jianbang2, jianbang3, jianbang4, jianbang5, jianbang6, jianbang7, jianbang8, jianbang9;
    shape_msgs::SolidPrimitive sphere10, sphere11, sphere12, sphere13, sphere14, sphere15, sphere16, sphere17, sphere18;

    sphere10.dimensions.resize ( 1 );
    sphere10.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere10.type=shape_msgs::SolidPrimitive::SPHERE;
    jb_vec = p1 + 0.1 * p1p2;
    jianbang1.linear() = sphere_rotation;
    jianbang1.translation() = jb_vec;
    
    sphere11.dimensions.resize ( 1 );
    sphere11.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere11.type=shape_msgs::SolidPrimitive::SPHERE;
    jb_vec = p1 + 0.2 * p1p2;
    jianbang2.linear() = sphere_rotation;
    jianbang2.translation() = jb_vec;
    
    sphere12.dimensions.resize ( 1 );
    sphere12.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere12.type=shape_msgs::SolidPrimitive::SPHERE;
    jb_vec = p1 + 0.3 * p1p2;
    jianbang3.linear() = sphere_rotation;
    jianbang3.translation() = jb_vec;
    
    sphere13.dimensions.resize ( 1 );
    sphere13.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere13.type=shape_msgs::SolidPrimitive::SPHERE;
    jb_vec = p1 + 0.4 * p1p2;
    jianbang4.linear() = sphere_rotation;
    jianbang4.translation() = jb_vec;
    
    sphere14.dimensions.resize ( 1 );
    sphere14.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere14.type=shape_msgs::SolidPrimitive::SPHERE;
    jb_vec = p1 + 0.5 * p1p2;
    jianbang5.linear() = sphere_rotation;
    jianbang5.translation() = jb_vec;
    
    sphere15.dimensions.resize ( 1 );
    sphere15.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere15.type=shape_msgs::SolidPrimitive::SPHERE;
    jb_vec = p1 + 0.6 * p1p2;
    jianbang6.linear() = sphere_rotation;
    jianbang6.translation() = jb_vec;
    
    sphere16.dimensions.resize ( 1 );
    sphere16.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere16.type=shape_msgs::SolidPrimitive::SPHERE;
    jb_vec = p1 + 0.7 * p1p2;
    jianbang7.linear() = sphere_rotation;
    jianbang7.translation() = jb_vec;
    
    sphere17.dimensions.resize ( 1 );
    sphere17.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere17.type=shape_msgs::SolidPrimitive::SPHERE;
    jb_vec = p1 + 0.8 * p1p2;
    jianbang8.linear() = sphere_rotation;
    jianbang8.translation() = jb_vec;
    
    sphere18.dimensions.resize ( 1 );
    sphere18.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere18.type=shape_msgs::SolidPrimitive::SPHERE;
    jb_vec = p1 + 0.9 * p1p2;
    jianbang9.linear() = sphere_rotation;
    jianbang9.translation() = jb_vec;

    // // 绘制大臂
    Eigen::Vector3d p2p3 = p3 - p2;
    Eigen::Vector3d db_vec;
    Eigen::Affine3d dabi1, dabi2, dabi3, dabi4, dabi5, dabi6, dabi7, dabi8, dabi9;
    shape_msgs::SolidPrimitive sphere19, sphere20, sphere21, sphere22, sphere23, sphere24, sphere25, sphere26, sphere27;

    sphere19.dimensions.resize ( 1 );
    sphere19.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere19.type=shape_msgs::SolidPrimitive::SPHERE;
    db_vec = p2 + 0.1 * p2p3;
    dabi1.linear() = sphere_rotation;
    dabi1.translation() = db_vec;
    
    sphere20.dimensions.resize ( 1 );
    sphere20.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere20.type=shape_msgs::SolidPrimitive::SPHERE;
    db_vec = p2 + 0.2 * p2p3;
    dabi2.linear() = sphere_rotation;
    dabi2.translation() = db_vec;
    
    sphere21.dimensions.resize ( 1 );
    sphere21.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere21.type=shape_msgs::SolidPrimitive::SPHERE;
    db_vec = p2 + 0.3 * p2p3;
    dabi3.linear() = sphere_rotation;
    dabi3.translation() = db_vec;
    
    sphere22.dimensions.resize ( 1 );
    sphere22.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere22.type=shape_msgs::SolidPrimitive::SPHERE;
    db_vec = p2 + 0.4 * p2p3;
    dabi4.linear() = sphere_rotation;
    dabi4.translation() = db_vec;
    
    sphere23.dimensions.resize ( 1 );
    sphere23.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere23.type=shape_msgs::SolidPrimitive::SPHERE;
    db_vec = p2 + 0.5 * p2p3;
    dabi5.linear() = sphere_rotation;
    dabi5.translation() = db_vec;
    
    sphere24.dimensions.resize ( 1 );
    sphere24.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere24.type=shape_msgs::SolidPrimitive::SPHERE;
    db_vec = p2 + 0.6 * p2p3;
    dabi6.linear() = sphere_rotation;
    dabi6.translation() = db_vec;
    
    sphere25.dimensions.resize ( 1 );
    sphere25.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere25.type=shape_msgs::SolidPrimitive::SPHERE;
    db_vec = p2 + 0.7 * p2p3;
    dabi7.linear() = sphere_rotation;
    dabi7.translation() = db_vec;
    
    sphere26.dimensions.resize ( 1 );
    sphere26.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere26.type=shape_msgs::SolidPrimitive::SPHERE;
    db_vec = p2 + 0.8 * p2p3;
    dabi8.linear() = sphere_rotation;
    dabi8.translation() = db_vec;
    
    sphere27.dimensions.resize ( 1 );
    sphere27.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere27.type=shape_msgs::SolidPrimitive::SPHERE;
    db_vec = p2 + 0.9 * p2p3;
    dabi9.linear() = sphere_rotation;
    dabi9.translation() = db_vec;

    // // 绘制小臂
    Eigen::Vector3d p3p4 = p4 - p3;
    Eigen::Vector3d xb_vec;
    Eigen::Affine3d xb1, xb2, xb3, xb4, xb5, xb6, xb7, xb8, xb9;
    shape_msgs::SolidPrimitive sphere28, sphere29, sphere30, sphere31, sphere32, sphere33, sphere34, sphere35, sphere36;

    sphere28.dimensions.resize ( 1 );
    sphere28.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere28.type=shape_msgs::SolidPrimitive::SPHERE;
    xb_vec = p3 + 0.1 * p3p4;
    xb1.linear() = sphere_rotation;
    xb1.translation() = xb_vec;

    sphere29.dimensions.resize ( 1 );
    sphere29.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere29.type=shape_msgs::SolidPrimitive::SPHERE;
    xb_vec = p3 + 0.2 * p3p4;
    xb2.linear() = sphere_rotation;
    xb2.translation() = xb_vec;

    sphere30.dimensions.resize ( 1 );
    sphere30.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere30.type=shape_msgs::SolidPrimitive::SPHERE;
    xb_vec = p3 + 0.3 * p3p4;
    xb3.linear() = sphere_rotation;
    xb3.translation() = xb_vec;

    sphere31.dimensions.resize ( 1 );
    sphere31.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere31.type=shape_msgs::SolidPrimitive::SPHERE;
    xb_vec = p3 + 0.4 * p3p4;
    xb4.linear() = sphere_rotation;
    xb4.translation() = xb_vec;

    sphere32.dimensions.resize ( 1 );
    sphere32.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere32.type=shape_msgs::SolidPrimitive::SPHERE;
    xb_vec = p3 + 0.5 * p3p4;
    xb5.linear() = sphere_rotation;
    xb5.translation() = xb_vec;

    sphere33.dimensions.resize ( 1 );
    sphere33.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere33.type=shape_msgs::SolidPrimitive::SPHERE;
    xb_vec = p3 + 0.6 * p3p4;
    xb6.linear() = sphere_rotation;
    xb6.translation() = xb_vec;

    sphere34.dimensions.resize ( 1 );
    sphere34.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere34.type=shape_msgs::SolidPrimitive::SPHERE;
    xb_vec = p3 + 0.7 * p3p4;
    xb7.linear() = sphere_rotation;
    xb7.translation() = xb_vec;

    sphere35.dimensions.resize ( 1 );
    sphere35.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere35.type=shape_msgs::SolidPrimitive::SPHERE;
    xb_vec = p3 + 0.8 * p3p4;
    xb8.linear() = sphere_rotation;
    xb8.translation() = xb_vec;

    sphere36.dimensions.resize ( 1 );
    sphere36.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.05;
    sphere36.type=shape_msgs::SolidPrimitive::SPHERE;
    xb_vec = p3 + 0.9 * p3p4;
    xb9.linear() = sphere_rotation;
    xb9.translation() = xb_vec;

    test_node.addCollisionObject ( sphere1, qugan1, 0 );
    test_node.addCollisionObject ( sphere2, qugan2, 1 );
    test_node.addCollisionObject ( sphere3, qugan3, 2 );
    test_node.addCollisionObject ( sphere4, qugan4, 3 );
    test_node.addCollisionObject ( sphere5, qugan5, 4 );
    test_node.addCollisionObject ( sphere6, qugan6, 5 );
    test_node.addCollisionObject ( sphere7, qugan7, 6 );
    test_node.addCollisionObject ( sphere8, qugan8, 7 );
    test_node.addCollisionObject ( sphere9, qugan9, 8 );
    test_node.addCollisionObject ( sphere10, jianbang1, 9 );
    test_node.addCollisionObject ( sphere11, jianbang2, 10 );
    test_node.addCollisionObject ( sphere12, jianbang3, 11 );
    test_node.addCollisionObject ( sphere13, jianbang4, 12 );
    test_node.addCollisionObject ( sphere14, jianbang5, 13 );
    test_node.addCollisionObject ( sphere15, jianbang6, 14 );
    test_node.addCollisionObject ( sphere16, jianbang7, 15 );
    test_node.addCollisionObject ( sphere17, jianbang8, 16 );
    test_node.addCollisionObject ( sphere18, jianbang9, 17 );
    test_node.addCollisionObject ( sphere19, dabi1, 18 );
    test_node.addCollisionObject ( sphere20, dabi2, 19 );
    test_node.addCollisionObject ( sphere21, dabi3, 20 );
    test_node.addCollisionObject ( sphere22, dabi4, 21 );
    test_node.addCollisionObject ( sphere23, dabi5, 22 );
    test_node.addCollisionObject ( sphere24, dabi6, 23 );
    test_node.addCollisionObject ( sphere25, dabi7, 24 );
    test_node.addCollisionObject ( sphere26, dabi8, 25 );
    test_node.addCollisionObject ( sphere27, dabi9, 26 );
    test_node.addCollisionObject ( sphere28, xb1, 27 );
    test_node.addCollisionObject ( sphere29, xb2, 28 );
    test_node.addCollisionObject ( sphere30, xb3, 29 );
    test_node.addCollisionObject ( sphere31, xb4, 30 );
    test_node.addCollisionObject ( sphere32, xb5, 31 );
    test_node.addCollisionObject ( sphere33, xb6, 32 );
    test_node.addCollisionObject ( sphere34, xb7, 33 );
    test_node.addCollisionObject ( sphere35, xb8, 34 );
    test_node.addCollisionObject ( sphere36, xb9, 35 );

    test_node.addCollisionObject ( sphere_pw, pw_Affine3d, 50 );
    test_node.addCollisionObject ( sphere_p1, p1_Affine3d, 51 );
    test_node.addCollisionObject ( sphere_p2, p2_Affine3d, 52 );
    test_node.addCollisionObject ( sphere_p3, p3_Affine3d, 53 );
    test_node.addCollisionObject ( sphere_p4, p4_Affine3d, 54 );
    test_node.displayObjects();
    /******************************************************************************************************************************/

    // ros::spin();

    return 0;
}