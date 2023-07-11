#include <iostream>
// C++启动shell脚本所需头文件
#include <stdio.h>   //popen()
#include <string.h>  //memset()

using namespace std;

int main(int argc, char **argv)
{
  setlocale(LC_ALL,""); // 中文输出
  cout << "大灰程序一键启动！" << endl;
  
  /**************************************分别打开终端，启动程序！*************************************/
  system("gnome-terminal -e /home/yu/big_grey/src/shell/big_grey_brain.sh");
  system("gnome-terminal -e /home/yu/big_grey/src/shell/manipulator_init.sh");
  system("gnome-terminal -e /home/yu/big_grey/src/shell/car_init.sh");
  system("gnome-terminal -e /home/yu/big_grey/src/shell/waist_init.sh");
  // system("gnome-terminal -e /home/yu/big_grey/src/shell/send_apple_site.sh");
  system("gnome-terminal -e /home/yu/big_grey/src/shell/publish_pointcloud.sh");
  system("gnome-terminal -e /home/yu/big_grey/src/shell/fabrik_demo.sh");
  system("gnome-terminal -e /home/yu/big_grey/src/shell/grasp.sh");
  /************************************************************************************************************/

  return 0;
}
