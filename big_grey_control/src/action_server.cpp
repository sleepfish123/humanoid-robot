/****** Action服务端所需头文件 ******/
#include <ros/ros.h>
#include <iostream>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
// #include "actionlib/server/action_server.h" // action服务端的相关定义，请加入到驱动节点的头文件中
#include "actionlib/server/simple_action_server.h"
#include "actionlib/server/server_goal_handle.h"// action服务端的目标控制句柄定义，与接收的目标相关联后，可以用来实现action的信息反馈等操作       
#include <std_msgs/Float64MultiArray.h> // 传递路点中各关节弧度
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
// #include<string.h>
#include<string>
#include <std_msgs/String.h>
#include <unistd.h> // 延时函数头文件 sleep(5); // 延迟5秒

using namespace std;

#define PI 3.1415926
// 弧度值转角度值
float rad2deg(float rad) {
    float theta;
    theta = rad / PI * 180;
    return theta;
}

class RightArmFollowJointTrajectory {
private:
    ros::NodeHandle n_as;
    std::string action_name_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>  as_; // 定义action服务端

    // 右臂规划组各关节顺序
    string joints_seq[9] = {"bend_joint", "foot_joint", "right_big_arm_joint", "right_elbow_joint", "right_ffside_joint", 
                                            "right_front_back_joint", "right_small_arm_joint", "swing_joint", "up_down_joint"};

    control_msgs::FollowJointTrajectoryGoal goal_;
    control_msgs::FollowJointTrajectoryFeedback feedback_;
    control_msgs::FollowJointTrajectoryResult result_;  // 用来反馈action目标的执行情况，客户端由此可以得知服务端是否执行成功了

    sensor_msgs::JointState joint_states_;
    // 发布/joint_states
    ros::Publisher joint_states_pub = n_as.advertise<sensor_msgs::JointState>("/joint_states", 100);
    // actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> goal_handle_; // 定义action服务端目标控制句柄

public:
    // 构造函数
    RightArmFollowJointTrajectory(std::string name):
        // 定义服务器
        as_(n_as, name, boost::bind(&RightArmFollowJointTrajectory::goalCB, this, _1), false),
        action_name_(name) {
        as_.registerPreemptCallback(boost::bind(&RightArmFollowJointTrajectory::preemptCB, this)); // 注册preempt回调函数
        as_.start(); // 服务器开始运行
        cout << "/****** 构造函数调用完毕，服务器开始运行 ******/" << endl;
    } 
    // void goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
    void goalCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
    {
        ROS_INFO("进入goal回调函数");
        ros::Rate r(1); // 以1Hz的速率发布
        // bool success = true;

        // 将各路点的各关节角度值储存起来
        trajectory_msgs::JointTrajectory small_ju_trajectory;
        small_ju_trajectory = save_trajectory(goal->trajectory);
        int points_number = small_ju_trajectory.points.size(); // 路点数量
        joint_states_.name = small_ju_trajectory.joint_names;
        joint_states_.position = small_ju_trajectory.points[points_number-1].positions; // rviz中只显示最后一个路点的各关节角度值
        joint_states_.header.stamp=ros::Time::now();
        joint_states_pub.publish(joint_states_);
        r.sleep();

        ROS_INFO("rviz中显示最后一个路点的各关节角度值(joint_states_.position)：");
        for (int q=0; q<9; q++) {
            cout << joints_seq[q] << "：" << rad2deg(joint_states_.position[q]) << "\t";
        }
        cout << endl;

        // action完成后，向客户端返回结果
        // result_.error_code = 0;
        // set the action state to succeeded
        // as_.setSucceeded(result_);
        // as_.setSucceeded(); // actionlib::SimpleActionServer有而actionlib::ActionServer没有
        ROS_INFO("%s succeeded", action_name_.c_str()); // C中没有string类型，故必须通过string类对象的成员函数c_str()把string对象转换成C中的char样式
    }

    // 此函数将各路点的各关节角度值储存起来，返回trajectory_msgs::JointTrajectory类型
    trajectory_msgs::JointTrajectory save_trajectory(trajectory_msgs::JointTrajectory trajectory) {
        trajectory_msgs::JointTrajectory small_ju_trajectory_1;
        small_ju_trajectory_1 = trajectory;
        int points_number_1;
        points_number_1 = small_ju_trajectory_1.points.size(); // 路点数量
        ROS_INFO("路点数量为：%i", points_number_1);
        // 9个关节，脚部1+腰部3+右臂5
        ROS_INFO("打印各关节名称");
        for (int n=0; n<9; n++) {
            cout << small_ju_trajectory_1.joint_names[n] << endl;
        }

        // float64 rad[points_number][9]; // 定义一个二维数组，储存关节弧度值，行数为路点数，列数为关节数
        // float64 theta[points_number][9]; // 定义一个二维数组，储存关节角度值
        float rad;
        float theta_1;
        float theta[9];
        // 打印每个路点中的数据
        for (int j=0; j<points_number_1; j++) {
            ROS_INFO("第%i个路点：", j);
            // cout << "第" << j << "个路点：";
            cout << "打印弧度值：" << "\t";
            for (int k=0; k<9; k++) {
                cout << small_ju_trajectory_1.points[j].positions[k] << "\t";
                rad = small_ju_trajectory_1.points[j].positions[k];
                theta_1 = rad2deg(rad);
                theta[k] = theta_1;
            }
            cout << endl;

            cout << "打印角度值：" << "\t";
            for (int l=0; l<9; l++) {
                cout << theta[l] << "\t";
            }
            cout << endl;
        }

        return small_ju_trajectory_1;
    }

    // 设置优先抢占处理，创建抢占回调以确保操作快速响应取消请求
    void preemptCB() {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
    }

    // 析构函数
    ~RightArmFollowJointTrajectory() {
        cout << "/****** 析构函数调用完毕 ******/" << endl;
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,""); // 中文输出
    ros::init(argc, argv, "action_server_node");
    ROS_INFO_STREAM("开整！");
    RightArmFollowJointTrajectory RAFJT("small_ju/follow_joint_trajectory");

    ros::spin();
    return 0;
}



