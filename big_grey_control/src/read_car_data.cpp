#include <iostream>
#include <serial/serial.h>
#include <ros/ros.h>
#include <unistd.h> // 延时函数头文件 sleep(5); // 延迟5秒
#include <string>

using namespace std;

void read_servo_data(int id_, uint8_t *read_data_) {
    read_data_[0] = 0xff;
    read_data_[1] = 0xff;
    read_data_[2] = id_; // 舵机ID号
    read_data_[3] = 0x04; // 有效数据长度
    read_data_[4] = 0x02; // 读指令
    read_data_[5] = 0x38; // 从地址0x2a（舵机首地址）处
    read_data_[6] = 0x02; // 读2个字节

    int check_sum = 0;
    // 计算校验和，计算方式见SC515产品手册
    for(int i=2; i<7; i++)
    {
        check_sum = check_sum + read_data_[i];
    }    
    read_data_[7] = ~(char(check_sum));
}

int main(int argc, char**argv) {
    setlocale(LC_ALL,""); // 中文输出
    serial::Serial ser;

    int servo_id_r_elbow = 7; // right_elbow_joint
    int servo_id_r_cetai = 8; // right_ffside_joint

    float init_data_r_elbow = 790; // right_elbow_joint
    float init_data_r_cetai = 485; // right_ffside_joint
    
    uint8_t read_data_r_elbow[8];
    size_t read_data_length_r_elbow = 8;
    uint8_t result_r_elbow[8];
    size_t result_length_r_elbow = 8;

    uint8_t read_data_r_cetai[8];
    size_t read_data_length_r_cetai = 8;
    uint8_t result_r_cetai[8];
    size_t result_length_r_cetai = 8;

    uint16_t a;
    uint16_t b;

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

    read_servo_data(servo_id_r_elbow, read_data_r_elbow);
    cout << "输出read_data_r_elbow：";
    for (int j1=0; j1<8; j1++) {
        printf("%04x\t", read_data_r_elbow[j1]);
    }
    cout << endl;
    read_servo_data(servo_id_r_cetai, read_data_r_cetai);
    cout << "输出read_data_r_cetai：";
    for (int j2=0; j2<8; j2++) {
        printf("%04x\t", read_data_r_cetai[j2]);
    }
    cout << endl;

    while (ser.isOpen()) {
        ser.write(read_data_r_elbow, read_data_length_r_elbow);
        sleep(1);
        ser.read(result_r_elbow, result_length_r_elbow); // available()返回缓冲区中的字符数
        sleep(1);
        ser.write(read_data_r_cetai, read_data_length_r_cetai);
        sleep(1);
        ser.read(result_r_cetai, result_length_r_cetai); // available()返回缓冲区中的字符数
        sleep(1);

        cout << "result_r_elbow：";
        for (int k1=0; k1<8; k1++) {
            printf("%04x\t", result_r_elbow[k1]);
        }
        cout << endl;
        cout << "result_r_cetai：";
        for (int k2=0; k2<8; k2++) {
            printf("%04x\t", result_r_cetai[k2]);
        }
        cout << endl;

        int r_elbow_result = (result_r_elbow[5] << 8) + result_r_elbow[6];
        // int c1 = result_r_elbow[5]; int d1 = result_r_elbow[6];
        // a = c1 << 8;
        // a = a + d1;
        uint16_t c1 = 0;
        c1 = result_r_elbow[6] | (result_r_elbow[5] << 8);
        printf("c1 : %04x\n", c1);
        printf("r_elbow_result（十六进制）：%04x\n", r_elbow_result);
        printf("r_elbow_result（十进制）：%d\n", r_elbow_result);
        int r_cetai_result = (result_r_cetai[5] << 8) + result_r_cetai[6];
        // int c2 = result_r_cetai[5]; int d2 = result_r_cetai[6];
        // b = c2 << 8;
        // b = b + d2;
        // printf("b : %04x\n", b);
        uint16_t c2 = 0;
        c2 = result_r_cetai[6] | (result_r_cetai[5] << 8);
        printf("c2 : %04x\n", c2);
        printf("r_cetai_result（十六进制）：%04x\n", r_cetai_result);
        printf("r_cetai_result（十进制）：%d\n", r_cetai_result);

        // 舵机值转化为角度值
        float r_elbow_theta = (c1 - init_data_r_elbow) / (-3.4);
        printf("读出的角度值：%f\n", r_elbow_theta);
        // 舵机值转化为角度值
        float r_cetai_theta = (c2 - init_data_r_cetai) / (-3.4);
        printf("读出的角度值：%f\n", r_cetai_theta);
    }    

    return 0;
}
