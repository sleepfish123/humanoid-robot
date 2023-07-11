#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>

#include "big_grey_control/motor_public.h"
#include "big_grey_control/motor_motorcommand.h"

using namespace std;

serial::Serial ser; // 声明串口对象

int main(int argc, char** argv) {
    setlocale(LC_ALL,""); // 中文输出
    ros::init(argc, argv, "serial_control_node");
    ros::NodeHandle n_sc;
    // 订阅主题，并配置回调函数 
    // ros::Subscriber control_signal_sub = n_sc.subscribe("/joint_states", 10, servo_control_callback);

    try { 
        // 设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB0"); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) { 
        ROS_ERROR_STREAM("串口无法打开"); 
        return -1; 
    } 

    // 检测串口是否已经打开，并给出提示信息 
    if (ser.isOpen()) { 
        ROS_INFO_STREAM("串口已打开，初始化成功！"); 
    } 
    else { 
        ROS_ERROR_STREAM("串口无法打开，初始化失败！"); 
        return -1; 
    }

    // ros::Rate loop_rate(100);
    ser.flush();
    while (ros::ok())
    {
        cout << "请选择发送什么指令" << endl;
        // 打印要发送的命令
        Public::help();
        int choice = 0;
        cin >> choice;
        cin.get();

        switch (choice) {
        case 1 : {
            cout << "增量位置闭环控制 : 0xA8" << endl;
            cout << "请输入电机id号 : ";
            int id = 0;
            cin >> id;
            cin.get();
            uint8_t incrementalPositionControlData[MAITA_DATA_LEN] = {0};
            MotorCommand motorCommand;
            motorCommand.incrementalPositionControl(id, 100, 20, incrementalPositionControlData); // 速度：100度/s（内圈），角度：20度
            
            // 发送
            for (int i= 0; i < 50; i++) {
                // 发送
                ser.write(incrementalPositionControlData, MAITA_DATA_LEN);
                usleep(300000);

                // 读取
                uint8_t readIncrementalPositionControlData[MAITA_DATA_LEN] = {0};
                ser.read(readIncrementalPositionControlData, MAITA_DATA_LEN); // available()返回缓冲区中的字符数
                usleep(300000);
                
                Public::printCmd(MAITA_DATA_LEN, readIncrementalPositionControlData);
            }
            break;
        }
        case 2 : {
            cout << "查询指令 : 0x9c" << endl;
            cout << "请输入电机id号 : ";
            int id = 0;
            cin >> id;
            cin.get();
            uint8_t queryCmdData[10] = {0};
            MotorCommand motorCommand;
            motorCommand.queryCmd(id, queryCmdData);

            // 发送
            for (int i= 0; i < 50; i++) {
                ser.write(queryCmdData, MAITA_DATA_LEN);
                usleep(300000);

                // 读取
                uint8_t readQueryCmdData[10] = {0};
                ser.read(readQueryCmdData, MAITA_DATA_LEN); // available()返回缓冲区中的字符数
                usleep(300000);

                Public::printCmd(MAITA_DATA_LEN, readQueryCmdData);
            }
            break;
        }
        default:
            cout << "command is invalid !" << endl;
            break;
        }
    }

    ser.close();

    return 0;
}
