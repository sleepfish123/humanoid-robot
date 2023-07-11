#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>

using namespace std;

serial::Serial ser; // 声明串口对象

void query_msg(uint8_t *msg)
{
    // uint8_t action_tmp[6]={0};
    msg[0]=0x40;
    msg[1]=0x06;
    msg[0]=0x00;
    msg[1]=0x02;
//    msg[2]=0xfe;
//    msg[3]=0x02;
//    msg[4]=0x05;
//    msg[5]=0xfa;
//    memmove(action,action_tmp,sizeof(uint8_t)*6);
}

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
        cout << "输入一个数字，启动控制指令" << endl;
        int choice = 0;
        cin >> choice;
        cin.get();

        uint8_t query_cmd[4];
        query_msg(query_cmd);
        ser.write(query_cmd, 4);
        sleep(1);

        uint8_t recv_query_cmd[4];
        ser.read(recv_query_cmd, 4); // available()返回缓冲区中的字符数
        sleep(1);

        cout << "recv_query_cmd";
        for (int k1=0; k1<4; k1++) {
            printf("%04x\t", recv_query_cmd[k1]);
        }
        cout << endl;

        // cout << "result_r_cetai：";
        // for (int k2=0; k2<8; k2++) {
        //     printf("%04x\t", result_r_cetai[k2]);
        // }
        // cout << endl;

        // int r_elbow_result = (result_r_elbow[5] << 8) + result_r_elbow[6];
        // uint16_t c1 = 0;
        // c1 = result_r_elbow[6] | (result_r_elbow[5] << 8);
        // printf("c1 : %04x\n", c1);
        // printf("r_elbow_result（十六进制）：%04x\n", r_elbow_result);
        // printf("r_elbow_result（十进制）：%d\n", r_elbow_result);
        // int r_cetai_result = (result_r_cetai[5] << 8) + result_r_cetai[6];
        // // int c2 = result_r_cetai[5]; int d2 = result_r_cetai[6];
        // // b = c2 << 8;
        // // b = b + d2;
        // // printf("b : %04x\n", b);
        // uint16_t c2 = 0;
        // c2 = result_r_cetai[6] | (result_r_cetai[5] << 8);
        // printf("c2 : %04x\n", c2);
        // printf("r_cetai_result（十六进制）：%04x\n", r_cetai_result);
        // printf("r_cetai_result（十进制）：%d\n", r_cetai_result);

        // // 舵机值转化为角度值
        // float r_elbow_theta = (c1 - init_data_r_elbow) / (-3.4);
        // printf("读出的角度值：%f\n", r_elbow_theta);
        // // 舵机值转化为角度值
        // float r_cetai_theta = (c2 - init_data_r_cetai) / (-3.4);
        // printf("读出的角度值：%f\n", r_cetai_theta);
    }

    ser.close();

    return 0;
}
