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
    char aaa;
    cout << "Interrupt signal (" << signum << ") received.\n";
    brake(0,0);
    cout << "stop!!" << endl;
    
    Exit_Can();
    exit(signum);
}



// // 直线插值
// std::vector<std::array<float, 6>> linearInterpPos(const float *A, const float *B, int steps) {
//     std::vector<std::array<float, 6>> result;
//     for (int s = 0; s <= steps; ++s) {
//         float t = float(s) / steps;
//         std::array<float, 6> p;
//         for (int i = 0; i < 6; ++i)
//             p[i] = A[i] * (1 - t) + B[i] * t;
//         result.push_back(p);
//     }
//     return result;
// }

// // 多关节点执行函数
// void ACTmove_path(const std::vector<std::array<float, 6>> &joint_path, int deviceInd, int canInd) {
//     for (size_t k = 1; k < joint_path.size(); ++k) {
//         int npl[IDNUM];
//         uint32_t goto_jv[IDNUM];

//         for (int i = 0; i < IDNUM; ++i) {
//             goto_jv[i] = int(joint_path[k][i] * j2p);
//             npl[i] = int(((joint_path[k][i] - joint_path[k - 1][i]) * j2p / AG / n2p)/5);
//         }
//         setn(npl, deviceInd, canInd);
//         sendCanCommand(deviceInd, canInd, IDNUM, canidList, SET_MOTOR_POSITION , goto_jv);
//         usleep(AG * 400000);  
//     }
// }
// // void ACTmove_path(const std::vector<std::array<float, 6>> &joint_path, int deviceInd, int canInd) {
// //     constexpr float control_freq = 50.0;  // 控制频率 50Hz
// //     constexpr float dt = 1.0 / control_freq;
// //     constexpr int max_npl = 1000;  // 限制速度上限

// //     for (size_t k = 1; k < joint_path.size(); ++k) {
// //         int npl[IDNUM];
// //         uint32_t goto_jv[IDNUM];

// //         for (int i = 0; i < IDNUM; ++i) {
// //             goto_jv[i] = int(joint_path[k][i] * j2p);

// //             float delta_rad = joint_path[k][i] - joint_path[k - 1][i];
// //             float delta_pulse = delta_rad * j2p;
// //             float speed_per_sec = delta_pulse / dt;
// //             npl[i] = int(speed_per_sec / n2p);

// //             // 限速
// //             if (npl[i] > max_npl) npl[i] = max_npl;
// //             if (npl[i] < -max_npl) npl[i] = -max_npl;
// //         }

// //         setn(npl, deviceInd, canInd);
// //         sendCanCommand(deviceInd, canInd, IDNUM, canidList, SET_MOTOR_POSITION, goto_jv);

// //         usleep(int(dt * 1e6)); // 控制频率，50Hz = 20ms
// //     }
// // }



// //运动
// bool line_move(const float A[6], const float B[6], int deviceInd, int canInd, int steps = 50) {
//     auto linear_path = linearInterpPos(A, B, steps);
//     std::vector<std::array<float, 6>> joint_path;

//     for (auto &p : linear_path) {
//         for (int i = 0; i < 6; ++i) TH.pos[i] = p[i];
//         if (!TH.backward_move()) {
//             std::cerr << "IK failed at step" << std::endl;
//             return false;
//         }

//         std::array<float, 6> j;
//         for (int i = 0; i < 6; ++i) j[i] = TH.j[i];
//         joint_path.push_back(j);
//     }

//     ACTmove_path(joint_path, deviceInd, canInd);
//     return true;
// }
// bool joint_move_linear(const float A_pos[6], const float B_pos[6], int deviceInd, int canInd, int steps = 50) {
//     for (int i = 0; i < 6; ++i) TH.pos[i] = A_pos[i];
//     if (!TH.backward_move()) return false;
//     float A_j[6];
//     for (int i = 0; i < 6; ++i) A_j[i] = TH.j[i];

//     for (int i = 0; i < 6; ++i) TH.pos[i] = B_pos[i];
//     if (!TH.backward_move()) return false;
//     float B_j[6];
//     for (int i = 0; i < 6; ++i) B_j[i] = TH.j[i];

//     std::vector<std::array<float, 6>> joint_path;
//     for (int s = 0; s <= steps; ++s) {
//         float t = float(s) / steps;
//         std::array<float, 6> p;
//         for (int i = 0; i < 6; ++i)
//             p[i] = A_j[i] * (1 - t) + B_j[i] * t;
//         joint_path.push_back(p);
//     }

//     ACTmove_path(joint_path, deviceInd, canInd);
//     return true;
// }



int main()
{
    NMAX = 1000;

    vector<string> productSerialNumbers = query_can();
    if (productSerialNumbers.empty())
    {
        cout << RED << "未找到任何 USB 设备，请插入设备后重试！" << RESET << endl;
        exit(0);
    }
    else
    {
        cout << CYAN << "找到的 CAN 设备序列号：" << RESET;
        for (const string &serialNumber : productSerialNumbers)
        {
            cout << CYAN << serialNumber << RESET << endl;
        }
    }

    string ip = ip_address();
    cout << MAGENTA << "ip=" << ip << RESET << endl;

    cout << "qweqwdsaradasd" << endl;
    signal(SIGINT, signalHandler);

    Start_Can();
    cout << "初始化CAN成功" << endl;

    // mechanical_arm_origin(0,0);

    // 1. 当前点 A
    float A_pos[6]={-7.98, -265.08, 270.65, 0,0 ,0};
    //2. 目标点 B
    float B_pos[6]={-7.98, -265.08, 170.65, 0,0,0};
    //3.目标点C
    float C_pos[6]={-189.58,-265.08,170.65,0,0,0};
    //4.目标点D
    float D_pos[6]={-189.58,-265.08,270.65,0,0,0};
    // float arr[6]={0,0,0,0,0,0};
    // current_pose(arr,0,0);
    // for(int i=0;i<6;i++)
    // {
    //     cout<<"arr="<<arr[i]<<" ";
    // }
    // cout <<endl;
    float arr_1[6]={10.4095,-199.245,104.37, 0.0607264, -0.157845, 1.53666 };
    // float arr_2[6]={19.814, -388.451 ,104.37 ,0.0607264, -0.157845, 1.53666 };
    // float arr_3[6]={10.4095, -288.451 ,104.37 ,0.0607264, -0.157845, 1.53666 };
    float arr_2[6]={10.4095, -288.451 ,104.37 ,0.0607264, -0.157845, 1.53666 };
    float arr_3[6]={10.4095, -388.451 ,104.37 ,0.0607264, -0.157845, 1.53666 };
    while(1){
        pos_move_linear(arr_1,arr_2,0,0,50,2);
        sleep(2);
        // usleep(500000);
        pos_move_linear(arr_2,arr_3,0,0,50,5);
        sleep(2);
        pos_move_linear(arr_3,arr_1,0,0,50,2);
        sleep(2);
        // usleep(500000);
    }
    // pos_move_linear(arr_3,arr_1,0,0,50,2);
    // usleep(500000);
    // pos_move_linear(arr_2,arr_3,0,0,50,2);
    // usleep(500000);
    // line_move(A_pos, B_pos, 0, 0);
    // while(1)
    // {
    //     line_move(A_pos, B_pos, 0, 0);
    //     sleep(1);
    //     line_move(B_pos, C_pos, 0, 0);
    //     sleep(1);
    //     line_move(C_pos, D_pos, 0, 0);
    //     sleep(1);
    //     line_move(D_pos, A_pos, 0, 0);
    //     sleep(1);
    // }
    // while(1)
    // {
    //     pos_move_linear(A_pos,B_pos,0,0,50,3);
    //     sleep(1);
    //     pos_move_linear(B_pos,C_pos,0,0,50,3);
    //     sleep(1);
    //     pos_move_linear(C_pos,D_pos,0,0,50,3);
    //     sleep(1);
    //     pos_move_linear(D_pos,A_pos,0,0,50,3);
    //     sleep(1);
    // }

    if(Exit_Can())
    {
        cout<<"退出CAN成功"<<endl;
    }

    return 0;
}
/*
// 逆运动解算器
bool robotArm1::backward_move(){
	float j0[6];copy_value(j,j0,6);
	float
	sy=sin(pos[3]),cy=cos(pos[3]),
	sp=sin(pos[4]),cp=cos(pos[4]),
	sr=sin(pos[5]),cr=cos(pos[5]);
	float d56_0[3]={sp*cr*cy + sr*sy,sp*sy*cr-sr*cy,cp*cr},P5_0[3];
	float X6_0[3]={cp*cy,sy*cp,-sp};
	vec_rescale(d56_0,len[4],d56_0);
	vec_subtraction(pos,d56_0,P5_0);
	
	float d15_0[3];
	vec_subtraction(P5_0,P_0[1],d15_0);
	float l15=vec_length(d15_0);
	if(l15>len[1]+len[2]+len[3] or l15<abs(len[1]-len[2]-len[3])) return false;
    float a135=solve_trangle(len[1],len[2]+len[3],l15);
    float a513=solve_trangle(l15,len[1],len[2]+len[3]);
    
    float n_0[3],n0_0[3]={1,0,0};
	int step=0;
	bool dft[3]={true,true,true};
	while(-1<step and step<3){
		if(step==0){
			if(dft[step])
				if(d15_0[0]==0 and d15_0[1]==0) copy_value(n0_0,n_0,3);
				else n_0[0]=-d15_0[1],n_0[1]=d15_0[0],n_0[2]=0;
			else n_0[0]=-n_0[0],n_0[1]=-n_0[1];
			j[0]=vec_angle(n0_0,n_0,n_0[1]);
			step++;
		}else if(step==1){
			float d15_1[3],d150_1[3]={0,0,1};
			fromS0toS1(d15_0,d15_1,false);
			float mid=vec_angle(d150_1,d15_1,d15_1[1]);
			if(l15==len[1]+len[2]+len[3]) {a513=0;a135=pi;dft[step]=false;}
			else if(l15==len[1]-len[2]-len[3]) {mid=0;a513=0;a135=0;dft[step]=false;}//mid 无穷多解
			
			
			if(dft[step]){
				j[1]=mid+a513;
            	j[2]=pi-a135;
            	if(j[1]>=pi) j[1]-=2*pi;
			}
			else{
            	j[1]=mid-a513;
    			j[2]=a135-pi;
    			if(j[1]<-pi) j[1]+=2*pi;
			}	
			step++;
		}else if(step==2){
			float d56_3[3],d56_4[3];
			fromS0toS1(d56_0,d56_3,false);
			fromS1toS2(d56_3,d56_4,false);
			fromS2toS3(d56_4,d56_3,false);
			if(d56_3[1]==0 and d56_3[2]==0) {j[3]=0;dft[step]=false;}
			else{
				float n_3[3]={0,-d56_3[2],d56_3[1]},n0_3[3]={0,0,1};
				if(!dft[step]) n_3[1]=-n_3[1],n_3[2]=-n_3[2];
				j[3]=vec_angle(n0_3,n_3,n_3[1]);
			}
			fromS3toS4(d56_3,d56_4,false);
			float d560_4[3]={0,0,1};
			j[4]=vec_angle(d560_4,d56_4,-d56_4[1]);
			float X6_5[3],X6_t[3],X60_5[3]={0,0,1};
			fromS0toS1(X6_0,X6_5,false);
			fromS1toS2(X6_5,X6_t,false);
			fromS2toS3(X6_t,X6_5,false);
			fromS3toS4(X6_5,X6_t,false);
			fromS4toS5(X6_t,X6_5,false);
		    j[5]=vec_angle(X60_5,X6_5,X6_5[1]);
			step=gotostep(step,!checkcalj(j0),dft);
		}
	}
	// cout<<"step="<<step<<endl;
	if(step==3){
		//if(j[4]>=pi/2) j[4]-=2*pi; 
		return true;
	}else{
		copy_value(j0,j,6);
		return false;
	} 
};
void setn(int npL[IDNUM],int deviceInd,int canInd)
{
    uint32_t unpL1[IDNUM], unpL2[IDNUM];
    for (int i = 0; i < IDNUM; i++)
    {
        unpL1[i] = abs(npL[i]);
        unpL2[i] = -abs(npL[i]);
    }
    sendCanCommand(deviceInd, canInd,6,canidList,SET_MOTOR_MAX_SPEED,unpL1);
    sendCanCommand(deviceInd, canInd,6,canidList,SET_MOTOR_MIN_SPEED,unpL2);
}

void ACTmove(float *a, float *b, float T0,int deviceInd, int canInd) // 实际运动
{
    float t = 0;
    int k, i;
    int AT = int(T0 / 2 / AG);
    uint32_t goto_jv[AT + 2][IDNUM];
    int npl[AT + 2][IDNUM];
    for (k = 0; k < AT + 1; k++)
    {	
        for (i = 0; i < IDNUM; i++)
        {
            goto_jv[k][i] = int(a[i] * sin(2 * pi * k * AG / T0 - pi / 2) + b[i]);
            if (k > 0)
            {
                npl[k][i] = int((int(goto_jv[k][i]) - int(goto_jv[k - 1][i])) / AG / n2p);
            }
        }
    }
    
    for (i = 0; i < IDNUM; i++)
    {
        goto_jv[AT + 1][i] = int(a[i] + b[i]);
        npl[AT + 1][i] = int((int(goto_jv[AT + 1][i]) - int(goto_jv[AT][i])) / AG / n2p);
    }
    for (int f = 0; f < 6; f++)
    {

        std::string value_str = std::to_string(npl[k][f]);
        writeDebugInfoToFile(__func__, value_str.c_str());
    }
    for (k = 1; k < AT + 2; k++)
    {
        setn(npl[k],deviceInd,canInd);
        sendCanCommand(deviceInd,canInd,6,canidList,SET_MOTOR_POSITION,goto_jv[k]);

        usleep(AG * 400000);
    }
}

void plan_move(int deviceInd,int canInd)
{
    uint32_t curt_jv[IDNUM]; 

    int p = 0, gap[IDNUM];
    float C, k, tm = 0;
    int i, t, im = 0;
    int32_t data[IDNUM];
    get_canidlist(canidList);

    sendSimpleCanCommand(deviceInd,canInd,6,canidList,GET_MOTOR_PERSIONS,data);//发送1字节的指令
    

    float a[IDNUM], b[IDNUM];
    for (i = 0; i < IDNUM; i++)
    {
        gap[i] = TH.j[i] * j2p - int(data[i]);
        a[i] = gap[i] / 2.0;
        b[i] = (TH.j[i] * j2p + int(data[i])) / 2.0;
    }
    for (i = 1; i < IDNUM; i++)
        if (abs(gap[i]) > abs(gap[im]))
            im = i;
    float T0 = pi * abs(gap[im]) / NMAX / n2p;
    ACTmove(a, b, T0,deviceInd,canInd);
}
bool move_to_pos(int deviceInd, int canInd)
{
    bool token = TH.backward_move();
    if (token)
    {
        printArrayDebugInfo(TH.j, (sizeof(TH.j) / sizeof(TH.j[0])), "TH.j"); // 将TH.j写入log文件
        plan_move(deviceInd,canInd);
    }
    writeDebugInfoToFile(__func__, "坐标运动.");
    return token;
}
// 机械臂坐标运动
bool pos_movement(int deviceInd,int canInd,const float *arr)
{
    cout<<"pos= ";
    for (int i = 0; i < 6; i++)
    {
        TH.pos[i] = arr[i];
        cout<<TH.pos[i]<<", ";
    }
    cout<<endl;
    return move_to_pos(deviceInd,canInd);
}
*/