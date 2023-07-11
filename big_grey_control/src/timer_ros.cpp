#include "ros/ros.h"
#include<iostream>
#include <big_grey_control/RobotCarControl.h>
#include<big_grey_control/RobotWaist.h>
#include<big_grey_control/Robot.h>

#include <ctime>

using namespace std;

#define SIZE_MSG	14
#define DATA_SIZE	8
#define HEADER_1	0xfb
#define HEADER_2	0x75

#define WAIST_POS_CMD	0x4f
#define WAIST_VEL_CMD	0x4e
#define CAR_POS_CMD	0x23
#define CAR_VEL_CMD	0x22

const uint8_t waist_pos[SIZE_MSG] = {0xfb, 0x75, 0x00, 0x4f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xef, 0x5c};
const uint8_t waist_vel[SIZE_MSG] = {0xfb, 0x75, 0x00, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xef, 0x5c};
const uint8_t car_pos[SIZE_MSG] = {0xfb, 0x75, 0x00, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xef, 0x5c};
const uint8_t car_vel[SIZE_MSG] = {0xfb, 0x75, 0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xef, 0x5c};
const uint8_t full_info = {0};

uint8_t read_data[SIZE_MSG] = {0};

void print_data(uint8_t *data, int count, string cmd) {
	cout << cmd << " data : " << endl;
	for (int i = 0; i < count; i++) {
		printf("%x\t", data[i]);
	}
	cout << endl;
	cout << "==================================="  << endl;
	return;
}

// send (int __fd, const void *__buf, size_t __n, int __flags);
// const size_t size_msg = 14;

class Test{
public:
  // void ROSInit(int argc, char **argv);
	// void callback(const ros::TimerEvent& time_event);

	void ROSInit(int argc, char **argv)
	{
		ros::init(argc, argv, "talker");
		ros::NodeHandle n;
		
		ros::Timer timer1 = n.createTimer(ros::Duration(0.1), &Test::callback,this);
		
		ros::spin();
		return;
	}
	
	void callback(const ros::TimerEvent& time_event)
	{
		clock_t startTime, endTime;
		startTime = clock(); // 计时开始

		int sockfd_waist=socket(PF_INET, SOCK_STREAM,0);
		int sockfd_car=socket(PF_INET, SOCK_STREAM,0);

		// 腰部
		RobotWaist RobotWaist_(sockfd_waist);

		write(sockfd_waist, waist_pos, SIZE_MSG);
		read(sockfd_waist, read_data, SIZE_MSG);
		if (read_data[0] == HEADER_1 && read_data[1] == HEADER_2) {
			if (read_data[3] == WAIST_POS_CMD) {
				print_data(read_data, DATA_SIZE, "WAIST_POS_CMD");
			}
		}
		memset(read_data, 0, SIZE_MSG);

		write(sockfd_waist, waist_vel, SIZE_MSG);
		read(sockfd_waist, read_data, SIZE_MSG);
		if (read_data[0] == HEADER_1 && read_data[1] == HEADER_2) {
			if (read_data[3] == WAIST_VEL_CMD) {
				print_data(read_data, DATA_SIZE, "WAIST_VEL_CMD");
			}
		}
		memset(read_data, 0, SIZE_MSG);

		// 车
		// int socketfd = socket(PF_INET, SOCK_STREAM, 0);
		RobotCarControl RobotCar_(sockfd_car);

		write(sockfd_car, car_pos, SIZE_MSG);
		read(sockfd_car, read_data, SIZE_MSG);
		if (read_data[0] == HEADER_1 && read_data[1] == HEADER_2) {
			if (read_data[3] == CAR_POS_CMD) {
				print_data(read_data, DATA_SIZE, "CAR_POS_CMD");
			}
		}
		memset(read_data, 0, SIZE_MSG);
		
		write(sockfd_car, car_vel, SIZE_MSG);
		read(sockfd_car, read_data, SIZE_MSG);
		if (read_data[0] == HEADER_1 && read_data[1] == HEADER_2) {
			if (read_data[3] == CAR_VEL_CMD) {
				print_data(read_data, DATA_SIZE, "CAR_VEL_CMD");
			}
		}
		memset(read_data, 0, SIZE_MSG);

		// ROS_INFO("Callback  triggered");
		endTime = clock(); // 计时结束
    cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
	}
};
 
// void callback1(const ros::TimerEvent& time_e)
// {
//   ROS_INFO("Callback 1 triggered");
//   //cout<<time_e.current_real<<endl;//当前触发的时间，参考上面的解释
// }
 
// void callback2(const ros::TimerEvent&)
// {
//   ROS_INFO("Callback 2 triggered");
// }
 
int main(int argc, char **argv)
{
  // ros::init(argc, argv, "timer_test");
  // ros::NodeHandle n;

	Test timer;
	timer.ROSInit(argc, argv);
 
  // ros::Timer timer1 = n.createTimer(ros::Duration(0.5), callback1);//0.1s运行一次callback1

	// while (1) {
	// 	cout << "main" << endl;
	// 	sleep(1);
	// }
  // ros::Timer timer2 = n.createTimer(ros::Duration(1.0), callback2);//1s运行一次callback2
 
  // ros::spin();
  
  return 0;
}