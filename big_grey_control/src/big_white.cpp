#include "ros/ros.h"
#include<iostream>

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

class Test{
public:
  // void ROSInit(int argc, char **argv);
	// void callback(const ros::TimerEvent& time_event);

	void ROSInit(int argc, char **argv)
	{
		ros::init(argc, argv, "talker");
		ros::NodeHandle n;
		
		ros::Timer timer1 = n.createTimer(ros::Duration(1), &Test::callback1, this);
		ros::Timer timer2 = n.createTimer(ros::Duration(2), &Test::callback2, this);
		
		ros::spin();
		return;
	}
	
	void callback1(const ros::TimerEvent& time_event)
	{
		clock_t startTime, endTime;
		startTime = clock(); // 计时开始

		cout << "callback1" << endl;

		// ROS_INFO("Callback  triggered");
		endTime = clock(); // 计时结束
    	cout << "The callback1 run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
	}

	void callback2(const ros::TimerEvent& time_event)
	{
		clock_t startTime, endTime;
		startTime = clock(); // 计时开始

		cout << "callback2" << endl;

		// ROS_INFO("Callback  triggered");
		endTime = clock(); // 计时结束
    	cout << "The callback2 run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
	}
};

int main(int argc, char **argv)
{
	Test timer;
	timer.ROSInit(argc, argv);
 
	// 
  
  return 0;
}