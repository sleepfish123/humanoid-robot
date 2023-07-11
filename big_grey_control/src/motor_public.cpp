#include "big_grey_control/motor_public.h"

#include <iostream>

// 打印指令
void Public::printCmd(int num, uint8_t *cmd) {
    for (int i = 0; i < num; i++) {
        printf("%x  ", cmd[i]);
    }
    std::cout << std::endl;
}

// 打印help
void Public::help() {
    std::cout << "1 : 增量位置闭环控制 : 0xA8" << std::endl;
    std::cout << "2 : 查询指令 : 0x9c" << std::endl;
}
