#ifndef MOTORCOMMAND_H
#define MOTORCOMMAND_H

#include <iostream>
#include <unistd.h>

const int MAITA_DATA_LEN = 10;

class MotorCommand {
public:
    MotorCommand();
    ~MotorCommand();

    // 增量位置闭环控制（0xA8）
    // 以恒定100度/s的速度（内圈，减速比1:6）
    void incrementalPositionControl(int id, uint16_t vel, uint16_t deg, uint8_t *incrementalPositionControlData);

    // 查询指令
    void queryCmd(int id, uint8_t* queryCmdData);
    
private:
};

#endif