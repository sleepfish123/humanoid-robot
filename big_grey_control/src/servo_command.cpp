/****** 各种函数，配置控制舵机的指令包 ******/
#include "big_grey_control/servo_command.h"

void scs_action_write(uint8_t *action)
{
    // 字头，连续收到两个 0xff ，表示有数据包到达
   action[0] = 0xff;
   action[1] = 0xff;

   action[2] = 0xfe; // ID 号 254 为广播 ID,若控制器发出的 ID 号为 254(0XFE),所有的舵机均接收指令,除 PING 指令外其它指令均不返回应答信息(多个舵机连接在总线上不能使用广播 PING指令)
   action[3] = 0x02; // 指令长度
   action[4] = 0x05; // 执行异步写指令 ACTION，0x05
   action[5] = 0xfa; // 校验和

    // printf("打印action_msg");
    // for(int i1=0; i1<6; i1++)
    // {
    //     printf("%d  ", action[i1]);
    // }
    // printf("\n");
}

void sms_action_write(uint8_t *action)
{
    // 字头，连续收到两个 0xff ，表示有数据包到达
   action[0] = 0xff;
   action[1] = 0xff;

   action[2] = 0xfe; // ID 号 254 为广播 ID,若控制器发出的 ID 号为 254(0XFE),所有的舵机均接收指令,除 PING 指令外其它指令均不返回应答信息(多个舵机连接在总线上不能使用广播 PING指令)
   action[3] = 0x02; // 指令长度
   action[4] = 0x05; // 执行异步写指令 ACTION，0x05
   action[5] = 0xfa; // 校验和

    // printf("打印action_msg");
    // for(int i2=0; i2<6; i2++)
    // {
    //     printf("%d  ", action[i2]);
    // }
    // printf("\n");
}

void scs_regwrite(uint8_t *regwrite_format, int id, float theta, float init_data) // 异步写
{
    int servo = theta * 3.413333 + init_data; // 解析值
    int check_sum = 0;

    // 字头，连续收到两个 0xff ，表示有数据包到达
    regwrite_format[0] = 0xff;
    regwrite_format[1] = 0xff;

    // 舵机ID号，0-253，对应0x00-0xfd
    regwrite_format[2] = id; // ID号为2

    // 数据长度：等于待发送的参数长度 N 加上 2，即“N+2”
    regwrite_format[3] = 0x09; // 写入数据长度N为6，N+3=9
    regwrite_format[4] = 0x04; // 异步写指令 REG WRITE，为0x04
    regwrite_format[5] = 0x2a; // 写入数据的首地址，即舵机首地址

    regwrite_format[6] = servo >> 8; // 写入的第一个数据，位置高字节
    printf("控制信号的解析值 : %d\n.", servo); // 打印输出控制信号的弧度值
    regwrite_format[7] = servo & 0xff; // 写入的第二个数据，位置低字节
    regwrite_format[8] = 0; // 写入的第三个数据，时间高字节
    regwrite_format[9] = 0; // 写入的第四个数据，时间低字节
    regwrite_format[10] = 0; // 写入的第五个数据，速度高字节
    regwrite_format[11] = 0x32; // 写入的第六个数据，速度低字节
                                                              // 最大速度为1000（十进制），测试过当速度大于200（十进制）时会烧毁舵机

    // 计算校验和，计算方式见SC515产品手册
    for(int j1=2; j1<12; j1++)
    {
        check_sum = check_sum + regwrite_format[j1];
    }    
    regwrite_format[12] = ~(char(check_sum));

    // printf("发出控制指令control_msg: ");
    // for(int j2=0; j2<13; j2++)
    // {
    //     printf("%d  ", regwrite_format[j2]);
    // } 
    // printf("\n ");
}

void sms_regwrite(uint8_t *regwrite_format, int id, float theta, float init_data) // 异步写
{
    int servo = theta * 11.377778 + init_data;
    int check_sum = 0;

    // 字头，连续收到两个 0xff ，表示有数据包到达
    regwrite_format[0] = 0xff;
    regwrite_format[1] = 0xff;

    // 舵机ID号，0-253，对应0x00-0xfd
    regwrite_format[2] = id; // ID号为2

    // 数据长度：等于待发送的参数长度 N 加上 2，即“N+2”
    regwrite_format[3] = 0x09; // 写入数据长度N为6，N+3=9
    regwrite_format[4] = 0x04; // 异步写指令 REG WRITE，为0x04
    regwrite_format[5] = 0x2a; // 写入数据的首地址，即舵机首地址

    regwrite_format[6] = servo & 0xff; // 写入的第一个数据，位置低字节
    printf("控制信号的解析值 : %d\n.", servo); // 打印输出控制信号的弧度值
    regwrite_format[7] = servo >> 8; // 写入的第二个数据，位置高字节
    regwrite_format[8] = 0; // 写入的第三个数据，时间低字节
    regwrite_format[9] = 0; // 写入的第四个数据，时间高字节
    regwrite_format[10] = 0x32; // 写入的第五个数据，速度低字节
    regwrite_format[11] = 0; // 写入的第六个数据，速度高字节
                                                              // 最大速度为1000（十进制），测试过当速度大于200（十进制）时会烧毁舵机

    // 计算校验和，计算方式见SC515产品手册
    for(int j3=2; j3<12; j3++)
    {
        check_sum = check_sum + regwrite_format[j3];
    }    
    regwrite_format[12] = ~(char(check_sum));

    // printf("发出控制指令control_msg: ");
    // for(int j4=0; j4<13; j4++)
    // {
    //     printf("%d  ", regwrite_format[j4]);
    // } 
    // printf("\n ");
}

size_t scs_control_servo(int id, float theta, uint8_t* control_msg, uint8_t* action_msg, float init_data) // 给角度值，换算成弧度值，配置控制信号，向串口写入数据。返回regwrite_data_length数据长度
{
    // float pi = 3.1415926;
    // float radian1 = angle * pi / 180; // 换算成弧度值

    // 定义舵机的控制信号，REGWRITE DATA(异步写)，2个字头，1个ID，1个长度，1个指令，7个参数，1个校验和
    control_msg[13] = {0};
    size_t regwrite_data_length = 13;  // size_t类型，储存一个整数，这里是serial包中ser.write()函数所要求的数据类型
   // 定义舵机的action信号，ACTION(执行异步写)，触发REG WRITE写入，2个字头，1个ID，1个长度，1个指令，无参数
    action_msg[6] = {0};
    size_t action_length = 6;

    // 发出舵机的控制信号
    scs_regwrite(control_msg, id, theta, init_data);
    scs_action_write(action_msg);

    return regwrite_data_length;
}

size_t sms_control_servo(int id, float theta, uint8_t* control_msg, uint8_t* action_msg, float init_data) // 给角度值，换算成弧度值，配置控制信号，向串口写入数据
{
    // float pi = 3.1415926;
    // float radian1 = angle * pi / 180; // 换算成弧度值

    // 定义舵机的控制信号，REGWRITE DATA(异步写)，2个字头，1个ID，1个长度，1个指令，7个参数，1个校验和
    control_msg[13] = {0};
    size_t regwrite_data_length = 13;  // size_t类型，储存一个整数，这里是serial包中ser.write()函数所要求的数据类型
   // 定义舵机的action信号，ACTION(执行异步写)，触发REG WRITE写入，2个字头，1个ID，1个长度，1个指令，无参数
    action_msg[6] = {0};
    size_t action_length = 6;

    // 发出舵机的控制信号
    sms_regwrite(control_msg, id, theta, init_data);
    sms_action_write(action_msg);

    return regwrite_data_length;
}