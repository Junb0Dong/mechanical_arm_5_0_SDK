#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>



 

#include "Ti5CAN_Driver.h"
#include "Ti5MOVE.h"
#include "Ti5BASIC.h"
#include "Ti5LOGIC.h"
#include "communication.h"
#include "mathfunc.h"
#include "tool.h"
#include "clamping_jaw.h"
#include "drag_drop.h"
#include "socket_service.h"
#include "example.h"
#include "key_event.h"


// 定义颜色转义序列
#define RESET "\033[0m"
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */

using namespace std;

struct Vec3 {
    float x, y, z;
};

struct Pose {
    Vec3 pos;
    Vec3 rpy;  // roll, pitch, yaw
};


string filename;
char device[] = "/dev/ttyUSB0"; 
std::string ip_test = "192.168.130.11";//socket通信测试参数


void signalHandler(int signum)
{
    cout << "Interrupt signal (" << signum << ") received.\n";
    brake(0,0);
    cout << "stop!!" << endl;
    
    Exit_Can();
    exit(signum);
}
int main()
{
    vector<string> productSerialNumbers = query_can();
    if (productSerialNumbers.empty())
    {
        cout << "未找到任何 USB 设备，请插入设备后重试！" << endl;
        exit(0);
    }
    else
    {
        cout << "找到的 CAN 设备序列号：";
        for (const string &serialNumber : productSerialNumbers)
        {
            cout << serialNumber << endl;
        }
    }

    // ctrl+c 刹车
    signal(SIGINT, signalHandler);

    Start_Can();
    cout << "初始化CAN成功" << endl;

    // mechanical_arm_origin(0, 0);
    // sleep(3);

    // 键盘控制
    // keyboard_controller(0, 0);

    // print information about the pose and joint angles
    float joint_angles[6];
    float pose[6];
    current_angle(joint_angles, 0, 0); // 获取当前角度
    current_pose(pose, 0, 0); // 获取当前姿态
    std::cout << "当前关节角度: ";
    for (int i = 0; i < 6; ++i)
    {
        std::cout << joint_angles[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "当前姿态: ";
    for (int i = 0; i < 6; ++i)
    {
        std::cout << pose[i] << " "; // 0-5: x, y, z, roll, pitch, yaw
    }
    std::cout << std::endl;

    if(Exit_Can())
    {
        cout<<"退出CAN成功"<<endl;
    }

    return 0;
}
