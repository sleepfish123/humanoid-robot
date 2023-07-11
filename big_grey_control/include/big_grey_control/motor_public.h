#ifndef PUBLIC_H
#define PUBLIC_H

/*
public公共类，存放公共接口
*/

#include <iostream>
#include <unistd.h>

class Public {
public:
    Public() {}
    ~Public() {}

    // 打印命令帧
    static void printCmd(int num, uint8_t *cmd);

    // 打印help
    static void help();
private:
};

#endif