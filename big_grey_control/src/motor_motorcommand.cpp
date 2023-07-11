#include "big_grey_control/motor_motorcommand.h"
#include "big_grey_control/motor_public.h"

MotorCommand::MotorCommand() {}

MotorCommand::~MotorCommand() {}

// 增量位置闭环控制（0xA8）
// 以恒定100度/s的速度（内圈，减速比1:6）
// 速度单位：度/s（内圈），角度单位：度 * 100
void MotorCommand::incrementalPositionControl(int id, uint16_t vel, uint16_t deg, uint8_t *incrementalPositionControlData) {
    // uint8_t incrementalPositionControlData[10] = {0};
    // 封装命令
    // 帧ID
    incrementalPositionControlData[0] = 0x40 | id;
    incrementalPositionControlData[1] = 0x01;
    // 数据帧
    incrementalPositionControlData[2] = 0xA8;
    incrementalPositionControlData[3] = 0x00;
    incrementalPositionControlData[4] = 0x64; // 速度低
    incrementalPositionControlData[5] = 0x00; // 速度高
    incrementalPositionControlData[6] = 0xD0; // 位置低
    incrementalPositionControlData[7] = 0x07;
    incrementalPositionControlData[8] = 0x00;
    incrementalPositionControlData[9] = 0x00; // 位置高

    // 打印命令
    Public::printCmd(MAITA_DATA_LEN, incrementalPositionControlData);
}

void MotorCommand::queryCmd(int id, uint8_t* queryCmdData) {
    // uint8_t queryCmdData[10] = {0};
    // 封装命令
    // 帧ID
    queryCmdData[0] = 0x40 | id;
    queryCmdData[1] = 0x01;
    // 数据帧
    queryCmdData[2] = 0x9C;
    queryCmdData[3] = 0x00;
    queryCmdData[4] = 0x00; // 
    queryCmdData[5] = 0x00; // 
    queryCmdData[6] = 0x00; // 
    queryCmdData[7] = 0x00;
    queryCmdData[8] = 0x00;
    queryCmdData[9] = 0x00; // 

    // 打印命令
    Public::printCmd(MAITA_DATA_LEN, queryCmdData);
}

