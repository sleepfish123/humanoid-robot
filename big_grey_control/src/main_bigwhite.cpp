#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>

using namespace std;

serial::Serial ser; // 声明串口对象

#define WAIST_CONTROL_TEST   1
#define ARM_CONTROL_TEST   0
// #define FORWARD        0x10
// #define VEL_CMD          0x06

void print_cmd(int num, uint8_t *vec) {
    for (int i = 0; i < num; i++) {
        printf("%x  ", vec[i]);
    }
    cout << endl;
}

int main(int argc, char** argv) {

#if WAIST_CONTROL_TEST
    uint8_t motor_cmd_msg_1[13]={0};
    uint8_t motor_vel_msg_1[8]={0};
    uint8_t motor_start_msg_1[8]={0};

    uint8_t motor_cmd_msg_2[13]={0};
    uint8_t motor_vel_msg_2[8]={0};
    uint8_t motor_start_msg_2[8]={0};

    uint8_t motor_cmd_msg_3[13]={0};
    uint8_t motor_vel_msg_3[8]={0};
    uint8_t motor_start_msg_3[8]={0};

    // motor_cmd_msg_1[0]=0x01; // ID
    // motor_cmd_msg_1[1]=0x10; // func_code
    // motor_cmd_msg_1[2]=0x00; // addr_1
    // motor_cmd_msg_1[3]=0x37; // addr_2
    // motor_cmd_msg_1[4]=0x00; // write_num_1
    // motor_cmd_msg_1[5]=0x02; // write_num_2
    // motor_cmd_msg_1[6]=0x04; // bytes_num
    // motor_cmd_msg_1[7]=0xFF; // content_1
    // motor_cmd_msg_1[8]=0xFF; // content_2
    // motor_cmd_msg_1[9]=0xFC; // content_3
    // motor_cmd_msg_1[10]=0x18; // content_4
    // motor_cmd_msg_1[11]=0xF0; // CRC_1
    // motor_cmd_msg_1[12]=0x73; // CRC_2

    // motor_vel_msg_1[0]=0x01; // ID
    // motor_vel_msg_1[1]=0x06; // func_code
    // motor_vel_msg_1[2]=0x00; // addr_1
    // motor_vel_msg_1[3]=0x36; // addr_2
    // motor_vel_msg_1[4]=0x00; // data_1
    // motor_vel_msg_1[5]=0x64; // data_2
    // motor_vel_msg_1[6]=0x68; // CRC_1
    // motor_vel_msg_1[7]=0x2F; // CRC_2

    // motor_start_msg_1[0]=0x01; // ID
    // motor_start_msg_1[1]=0x06; // func_code
    // motor_start_msg_1[2]=0x00; // addr_1
    // motor_start_msg_1[3]=0x4E; // addr_2
    // motor_start_msg_1[4]=0x00; // data_1
    // motor_start_msg_1[5]=0x05; // data_2
    // motor_start_msg_1[6]=0x29; // CRC_1
    // motor_start_msg_1[7]=0xDE; // CRC_2

    // motor_cmd_msg_2[0]=0x02; // ID
    // motor_cmd_msg_2[1]=0x10; // func_code
    // motor_cmd_msg_2[2]=0x00; // addr_1
    // motor_cmd_msg_2[3]=0x37; // addr_2
    // motor_cmd_msg_2[4]=0x00; // write_num_1
    // motor_cmd_msg_2[5]=0x02; // write_num_2
    // motor_cmd_msg_2[6]=0x04; // bytes_num
    // motor_cmd_msg_2[7]=0xFF; // content_1
    // motor_cmd_msg_2[8]=0xFF; // content_2
    // motor_cmd_msg_2[9]=0xFC; // content_3
    // motor_cmd_msg_2[10]=0x18; // content_4
    // motor_cmd_msg_2[11]=0xFF; // CRC_1
    // motor_cmd_msg_2[12]=0x37; // CRC_2

    // motor_vel_msg_2[0]=0x02; // ID
    // motor_vel_msg_2[1]=0x06; // func_code
    // motor_vel_msg_2[2]=0x00; // addr_1
    // motor_vel_msg_2[3]=0x36; // addr_2
    // motor_vel_msg_2[4]=0x00; // data_1
    // motor_vel_msg_2[5]=0x64; // data_2
    // motor_vel_msg_2[6]=0x68; // CRC_1
    // motor_vel_msg_2[7]=0x1C; // CRC_2

    // motor_start_msg_2[0]=0x02; // ID
    // motor_start_msg_2[1]=0x06; // func_code
    // motor_start_msg_2[2]=0x00; // addr_1
    // motor_start_msg_2[3]=0x4E; // addr_2
    // motor_start_msg_2[4]=0x00; // data_1
    // motor_start_msg_2[5]=0x05; // data_2
    // motor_start_msg_2[6]=0x29; // CRC_1
    // motor_start_msg_2[7]=0xED; // CRC_2

    // motor_cmd_msg_3[0]=0x03; // ID
    // motor_cmd_msg_3[1]=0x10; // func_code
    // motor_cmd_msg_3[2]=0x00; // addr_1
    // motor_cmd_msg_3[3]=0x37; // addr_2
    // motor_cmd_msg_3[4]=0x00; // write_num_1
    // motor_cmd_msg_3[5]=0x02; // write_num_2
    // motor_cmd_msg_3[6]=0x04; // bytes_num
    // motor_cmd_msg_3[7]=0xFF; // content_1
    // motor_cmd_msg_3[8]=0xFF; // content_2
    // motor_cmd_msg_3[9]=0xFC; // content_3
    // motor_cmd_msg_3[10]=0x18; // content_4
    // motor_cmd_msg_3[11]=0xFB; // CRC_1
    // motor_cmd_msg_3[12]=0xCB; // CRC_2

    // motor_vel_msg_3[0]=0x03; // ID
    // motor_vel_msg_3[1]=0x06; // func_code
    // motor_vel_msg_3[2]=0x00; // addr_1
    // motor_vel_msg_3[3]=0x36; // addr_2
    // motor_vel_msg_3[4]=0x00; // data_1
    // motor_vel_msg_3[5]=0x64; // data_2
    // motor_vel_msg_3[6]=0x69; // CRC_1
    // motor_vel_msg_3[7]=0xCD; // CRC_2

    // motor_start_msg_3[0]=0x03; // ID
    // motor_start_msg_3[1]=0x06; // func_code
    // motor_start_msg_3[2]=0x00; // addr_1
    // motor_start_msg_3[3]=0x4E; // addr_2
    // motor_start_msg_3[4]=0x00; // data_1
    // motor_start_msg_3[5]=0x05; // data_2
    // motor_start_msg_3[6]=0x28; // CRC_1
    // motor_start_msg_3[7]=0x3C; // CRC_2

    motor_cmd_msg_1[0]=0x01; // ID
    motor_cmd_msg_1[1]=0x10; // func_code
    motor_cmd_msg_1[2]=0x00; // addr_1
    motor_cmd_msg_1[3]=0x37; // addr_2
    motor_cmd_msg_1[4]=0x00; // write_num_1
    motor_cmd_msg_1[5]=0x02; // write_num_2
    motor_cmd_msg_1[6]=0x04; // bytes_num
    motor_cmd_msg_1[7]=0xFF; // content_1
    motor_cmd_msg_1[8]=0xFF; // content_2
    motor_cmd_msg_1[9]=0xFC; // content_3
    motor_cmd_msg_1[10]=0x18; // content_4
    motor_cmd_msg_1[11]=0xF0; // CRC_1
    motor_cmd_msg_1[12]=0x73; // CRC_2

    motor_vel_msg_1[0]=0x01; // ID
    motor_vel_msg_1[1]=0x06; // func_code
    motor_vel_msg_1[2]=0x00; // addr_1
    motor_vel_msg_1[3]=0x36; // addr_2
    motor_vel_msg_1[4]=0x00; // data_1
    motor_vel_msg_1[5]=0x64; // data_2
    motor_vel_msg_1[6]=0x68; // CRC_1
    motor_vel_msg_1[7]=0x2F; // CRC_2

    motor_start_msg_1[0]=0x01; // ID
    motor_start_msg_1[1]=0x06; // func_code
    motor_start_msg_1[2]=0x00; // addr_1
    motor_start_msg_1[3]=0x4E; // addr_2
    motor_start_msg_1[4]=0x00; // data_1
    motor_start_msg_1[5]=0x03; // data_2
    motor_start_msg_1[6]=0x29; // CRC_1
    motor_start_msg_1[7]=0xDE; // CRC_2

    motor_cmd_msg_2[0]=0x02; // ID
    motor_cmd_msg_2[1]=0x10; // func_code
    motor_cmd_msg_2[2]=0x00; // addr_1
    motor_cmd_msg_2[3]=0x37; // addr_2
    motor_cmd_msg_2[4]=0x00; // write_num_1
    motor_cmd_msg_2[5]=0x02; // write_num_2
    motor_cmd_msg_2[6]=0x04; // bytes_num
    motor_cmd_msg_2[7]=0xFF; // content_1
    motor_cmd_msg_2[8]=0xFF; // content_2
    motor_cmd_msg_2[9]=0xFC; // content_3
    motor_cmd_msg_2[10]=0x18; // content_4
    motor_cmd_msg_2[11]=0xFF; // CRC_1
    motor_cmd_msg_2[12]=0x37; // CRC_2

    motor_vel_msg_2[0]=0x02; // ID
    motor_vel_msg_2[1]=0x06; // func_code
    motor_vel_msg_2[2]=0x00; // addr_1
    motor_vel_msg_2[3]=0x36; // addr_2
    motor_vel_msg_2[4]=0x00; // data_1
    motor_vel_msg_2[5]=0x64; // data_2
    motor_vel_msg_2[6]=0x68; // CRC_1
    motor_vel_msg_2[7]=0x1C; // CRC_2

    motor_start_msg_2[0]=0x02; // ID
    motor_start_msg_2[1]=0x06; // func_code
    motor_start_msg_2[2]=0x00; // addr_1
    motor_start_msg_2[3]=0x4E; // addr_2
    motor_start_msg_2[4]=0x00; // data_1
    motor_start_msg_2[5]=0x05; // data_2
    motor_start_msg_2[6]=0x29; // CRC_1
    motor_start_msg_2[7]=0xED; // CRC_2

    motor_cmd_msg_3[0]=0x03; // ID
    motor_cmd_msg_3[1]=0x10; // func_code
    motor_cmd_msg_3[2]=0x00; // addr_1
    motor_cmd_msg_3[3]=0x37; // addr_2
    motor_cmd_msg_3[4]=0x00; // write_num_1
    motor_cmd_msg_3[5]=0x02; // write_num_2
    motor_cmd_msg_3[6]=0x04; // bytes_num
    motor_cmd_msg_3[7]=0xFF; // content_1
    motor_cmd_msg_3[8]=0xFF; // content_2
    motor_cmd_msg_3[9]=0xFC; // content_3
    motor_cmd_msg_3[10]=0x18; // content_4
    motor_cmd_msg_3[11]=0xFB; // CRC_1
    motor_cmd_msg_3[12]=0xCB; // CRC_2

    motor_vel_msg_3[0]=0x03; // ID
    motor_vel_msg_3[1]=0x06; // func_code
    motor_vel_msg_3[2]=0x00; // addr_1
    motor_vel_msg_3[3]=0x36; // addr_2
    motor_vel_msg_3[4]=0x00; // data_1
    motor_vel_msg_3[5]=0x64; // data_2
    motor_vel_msg_3[6]=0x69; // CRC_1
    motor_vel_msg_3[7]=0xCD; // CRC_2

    motor_start_msg_3[0]=0x03; // ID
    motor_start_msg_3[1]=0x06; // func_code
    motor_start_msg_3[2]=0x00; // addr_1
    motor_start_msg_3[3]=0x4E; // addr_2
    motor_start_msg_3[4]=0x00; // data_1
    motor_start_msg_3[5]=0x05; // data_2
    motor_start_msg_3[6]=0x28; // CRC_1
    motor_start_msg_3[7]=0x3C; // CRC_2

    print_cmd(13, motor_cmd_msg_1);
    print_cmd(8, motor_vel_msg_1);
    print_cmd(8, motor_start_msg_1);

    print_cmd(13, motor_cmd_msg_2);
    print_cmd(8, motor_vel_msg_2);
    print_cmd(8, motor_start_msg_2);

    print_cmd(13, motor_cmd_msg_3);
    print_cmd(8, motor_vel_msg_3);
    print_cmd(8, motor_start_msg_3);
#endif

#if ARM_CONTROL_TEST
    uint8_t motor_cmd_msg_1[13]={0};
    uint8_t motor_vel_msg_1[8]={0};
    uint8_t motor_start_msg_1[8]={0};
#endif

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

#if WAIST_CONTROL_TEST
        cout << "正在上升（步进电机反转）" << endl;
        // if(ser.available()) {
            cout << "控制中" << endl;
            // ser_waist.flush();
            // 反转
            // uint8_t motor_cmd_msg_3[13]={0};
            // uint8_t motor_vel_msg_3[8]={0};
            // uint8_t motor_start_msg_3[8]={0};
            ser.write(motor_cmd_msg_1, 13);
            usleep(10000);
            ser.write(motor_vel_msg_1, 8);
            usleep(10000);
            ser.write(motor_start_msg_1, 8);
            usleep(10000);
            
            ser.write(motor_cmd_msg_2, 13);
            usleep(10000);
            ser.write(motor_vel_msg_2, 8);
            usleep(10000);
            ser.write(motor_start_msg_2, 8);
            usleep(10000);

            ser.write(motor_cmd_msg_3, 13);
            usleep(10000);
            ser.write(motor_vel_msg_3, 8);
            usleep(10000);
            ser.write(motor_start_msg_3, 8);
            usleep(10000);
        // }
#endif 
        // ros::spinOnce();
        // loop_rate.sleep();
        int flag = 0;
        cout << "输入: " << endl;
        cin >> flag;
    }

    ser.close();

    return 0;
}




// int main(int argc, char** argv)
// {
//     uint8_t motor_cmd_msg_1[13]={0};
//     uint8_t motor_vel_msg_1[8]={0};
//     uint8_t motor_start_msg_1[8]={0};

//     uint8_t motor_cmd_msg_2[13]={0};
//     uint8_t motor_vel_msg_2[8]={0};
//     uint8_t motor_start_msg_2[8]={0};

//     uint8_t motor_cmd_msg_3[13]={0};
//     uint8_t motor_vel_msg_3[8]={0};
//     uint8_t motor_start_msg_3[8]={0};

//     motor_cmd_msg_1[0]=0x01; // ID
//     motor_cmd_msg_1[1]=0x10; // func_code
//     motor_cmd_msg_1[2]=0x00; // addr_1
//     motor_cmd_msg_1[3]=0x37; // addr_2
//     motor_cmd_msg_1[4]=0x00; // write_num_1
//     motor_cmd_msg_1[5]=0x02; // write_num_2
//     motor_cmd_msg_1[6]=0x04; // bytes_num
//     motor_cmd_msg_1[7]=0xFF; // content_1
//     motor_cmd_msg_1[8]=0xFF; // content_2
//     motor_cmd_msg_1[9]=0xFC; // content_3
//     motor_cmd_msg_1[10]=0x18; // content_4
//     motor_cmd_msg_1[11]=0xF0; // CRC_1
//     motor_cmd_msg_1[12]=0x73; // CRC_2

//     motor_vel_msg_1[0]=0x01; // ID
//     motor_vel_msg_1[1]=0x06; // func_code
//     motor_vel_msg_1[2]=0x00; // addr_1
//     motor_vel_msg_1[3]=0x36; // addr_2
//     motor_vel_msg_1[4]=0x00; // data_1
//     motor_vel_msg_1[5]=0x64; // data_2
//     motor_vel_msg_1[6]=0x68; // CRC_1
//     motor_vel_msg_1[7]=0x2F; // CRC_2

//     motor_start_msg_1[0]=0x01; // ID
//     motor_start_msg_1[1]=0x06; // func_code
//     motor_start_msg_1[2]=0x00; // addr_1
//     motor_start_msg_1[3]=0x4E; // addr_2
//     motor_start_msg_1[4]=0x00; // data_1
//     motor_start_msg_1[5]=0x05; // data_2
//     motor_start_msg_1[6]=0x29; // CRC_1
//     motor_start_msg_1[7]=0xDE; // CRC_2

//     motor_cmd_msg_2[0]=0x02; // ID
//     motor_cmd_msg_2[1]=0x10; // func_code
//     motor_cmd_msg_2[2]=0x00; // addr_1
//     motor_cmd_msg_2[3]=0x37; // addr_2
//     motor_cmd_msg_2[4]=0x00; // write_num_1
//     motor_cmd_msg_2[5]=0x02; // write_num_2
//     motor_cmd_msg_2[6]=0x04; // bytes_num
//     motor_cmd_msg_2[7]=0xFF; // content_1
//     motor_cmd_msg_2[8]=0xFF; // content_2
//     motor_cmd_msg_2[9]=0xFC; // content_3
//     motor_cmd_msg_2[10]=0x18; // content_4
//     motor_cmd_msg_2[11]=0xFF; // CRC_1
//     motor_cmd_msg_2[12]=0x37; // CRC_2

//     motor_vel_msg_2[0]=0x02; // ID
//     motor_vel_msg_2[1]=0x06; // func_code
//     motor_vel_msg_2[2]=0x00; // addr_1
//     motor_vel_msg_2[3]=0x36; // addr_2
//     motor_vel_msg_2[4]=0x00; // data_1
//     motor_vel_msg_2[5]=0x64; // data_2
//     motor_vel_msg_2[6]=0x68; // CRC_1
//     motor_vel_msg_2[7]=0x1C; // CRC_2

//     motor_start_msg_2[0]=0x02; // ID
//     motor_start_msg_2[1]=0x06; // func_code
//     motor_start_msg_2[2]=0x00; // addr_1
//     motor_start_msg_2[3]=0x4E; // addr_2
//     motor_start_msg_2[4]=0x00; // data_1
//     motor_start_msg_2[5]=0x05; // data_2
//     motor_start_msg_2[6]=0x29; // CRC_1
//     motor_start_msg_2[7]=0xED; // CRC_2

//     motor_cmd_msg_3[0]=0x03; // ID
//     motor_cmd_msg_3[1]=0x10; // func_code
//     motor_cmd_msg_3[2]=0x00; // addr_1
//     motor_cmd_msg_3[3]=0x37; // addr_2
//     motor_cmd_msg_3[4]=0x00; // write_num_1
//     motor_cmd_msg_3[5]=0x02; // write_num_2
//     motor_cmd_msg_3[6]=0x04; // bytes_num
//     motor_cmd_msg_3[7]=0xFF; // content_1
//     motor_cmd_msg_3[8]=0xFF; // content_2
//     motor_cmd_msg_3[9]=0xFC; // content_3
//     motor_cmd_msg_3[10]=0x18; // content_4
//     motor_cmd_msg_3[11]=0xFB; // CRC_1
//     motor_cmd_msg_3[12]=0xCB; // CRC_2

//     motor_vel_msg_3[0]=0x03; // ID
//     motor_vel_msg_3[1]=0x06; // func_code
//     motor_vel_msg_3[2]=0x00; // addr_1
//     motor_vel_msg_3[3]=0x36; // addr_2
//     motor_vel_msg_3[4]=0x00; // data_1
//     motor_vel_msg_3[5]=0x64; // data_2
//     motor_vel_msg_3[6]=0x69; // CRC_1
//     motor_vel_msg_3[7]=0xCD; // CRC_2

//     motor_start_msg_3[0]=0x03; // ID
//     motor_start_msg_3[1]=0x06; // func_code
//     motor_start_msg_3[2]=0x00; // addr_1
//     motor_start_msg_3[3]=0x4E; // addr_2
//     motor_start_msg_3[4]=0x00; // data_1
//     motor_start_msg_3[5]=0x05; // data_2
//     motor_start_msg_3[6]=0x28; // CRC_1
//     motor_start_msg_3[7]=0x3C; // CRC_2

//     ros::init(argc, argv, "serial_port");
//     ros::NodeHandle n;

//     serial::Serial ser_waist("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000));

//     // if(ser_waist.isOpen()) {
//     //     ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
//     // } else {
//     //     return -1;
//     // }

//     ros::Rate loop_rate(100);
//     while (ros::ok())
//     {
//         if(ser_waist.available()) {
//             ser_waist.flush();
//             // 反转
//             // uint8_t motor_cmd_msg_3[13]={0};
//             // uint8_t motor_vel_msg_3[8]={0};
//             // uint8_t motor_start_msg_3[8]={0};
//             ser_waist.write(motor_cmd_msg_1, 13);
//             ser_waist.write(motor_vel_msg_1, 8);
//             ser_waist.write(motor_start_msg_1, 8);
//             usleep(100);
            
//             ser_waist.write(motor_cmd_msg_2, 13);
//             ser_waist.write(motor_vel_msg_2, 8);
//             ser_waist.write(motor_start_msg_2, 8);
//             usleep(100);

//             ser_waist.write(motor_cmd_msg_3, 13);
//             ser_waist.write(motor_vel_msg_3, 8);
//             ser_waist.write(motor_start_msg_3, 8);
//             usleep(100);
//         }

//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     ser_waist.close();
    
//     return 0;
// }

