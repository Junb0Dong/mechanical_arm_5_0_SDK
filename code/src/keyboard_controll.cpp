#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <cstdlib>

#include "Ti5MOVE.h"
#include "Ti5BASIC.h"
#include "Ti5LOGIC.h"
#include "communication.h"
#include "mathfunc.h"
#include "tool.h"
#include "clamping_jaw.h"
#include "drag_drop.h"

#include <csignal>

using namespace std;

string filename;
// string device_485_name;
char device[] = "/dev/ttyUSB0";

void signalHandler(int signum)
{

    char aaa;
    cout << "Interrupt signal (" << signum << ") received.\n";

    brake();
    cout << "stop!!" << endl;
    inspect_brake();
    logout();
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
    // string qqq=query_can();
    // cout<<"qqq="<<qqq<<endl;
    string ip = ip_address();
    cout << "ip=" << endl;

    // ctrl+c 刹车
    signal(SIGINT, signalHandler);
    login();
    cout << "login success" << endl;
    mechanical_arm_origin();
    sleep(3);
    brake();

    // 键盘控制
    mechanical_arm_origin();
    keyboard_controller();
    logout();
    return 0;
}
