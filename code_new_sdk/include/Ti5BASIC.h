#ifndef Ti5BASIC_H
#define Ti5BASIC_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <sstream>
#include <map>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <vector>
#include <regex>

#include "Ti5LOGIC.h"
#include "Ti5MOVE.h"



#define MAX_IP_ADDR_LEN 256 // 存储IP地址的最大长度
// #define LOG_PATH "/mnt/hgfs/mechanical_arm/log/daily_logger.log" //log路径
#define INI_PATH "../config.ini" // config.ini文件路径

extern float arr_s[6];

extern bool flag;

extern uint32_t reg_position_kp[6]; // 位置环比例
extern uint32_t reg_position_kd[6]; // 位置环微分

extern uint32_t reg_speed_kp[6]; // 速度环比例
extern uint32_t reg_speed_ki[6]; // 速度环积分

extern uint32_t reg_max_curr_value[6]; // 最大正电流
extern uint32_t reg_min_curr_value[6]; // 最小负电流

extern uint32_t reg_max_app_speed[6]; // 最大正向允许速度
extern uint32_t reg_min_app_speed[6]; // 最小负向允许速度

extern uint32_t reg_max_app_position[6]; // 电机最大正向位
extern uint32_t reg_min_app_position[6]; // 电机最大负向位

extern uint32_t electricity[6];        // 电流值
extern uint32_t electric_machinery[6]; // 电机错误状态
extern uint32_t reg_fault_clear[6];    // 清除电机错误
extern uint32_t ampere[6];                // 电机电流值

extern string log_path; // log文件

extern string SDKversion; // SDK version
extern int drag_value;

extern char LogInfo[100];//存储写入log文件的信息

#define BUFFER_SIZE 13 // uart通信
extern bool drag_read_data_flag;
extern "C"
{ // 添加extern "C"

  // 写入调试信息到文件
  void writeDebugInfoToFile(const char *func_name, const char *info);

  // 输出数组的调试信息
  void printArrayDebugInfo(float arr[], int size,const char *arr_name);

  // // 查询机械臂是否停止运动
  // bool inspect_brake();

  // void exit_progrem(); // 退出程序

  std::string SDK_version(); // SDK版本号

  std::string ip_address();

  // void loggerinfo(string content);//添加log

  void read_ini_information(); // 读取ini配置文件的内容
  
  // bool login(); // 登录can设备

  // bool logout(); // 登出can设备

  /*std::string query_can();
  查询can设备号
  */
  std::vector<std::string> query_can();

  // 获取电机参数
  void get_elc_info(int deviceInd,int canInd);

  /*设置电机参数
  参数:
    deviceInd：can设备号
    canInd：can通道号
    elc_parameterlist 要设置的对应参数
    parameterValue 要设置的参数值
  */
  void set_elc_info(int deviceInd,int canInd,uint8_t elc_parameterlist,uint32_t *parameterValue);
  
  /*设置电机最大正向允许速度
    参数：elc_parameterlist： 关节目标值
  */
  void set_max_speed_info(int deviceInd,int canInd,uint32_t *parameterValue);

  /*设置电机最大负向允许速度
    参数：elc_parameterlist： 关节目标值
  */
  void set_min_speed_info(int deviceInd,int canInd,uint32_t *parameterValue);

  /*获取电机电流值
    参数：
      deviceInd：can设备号
      canInd：can通道号
      data：电机电流值
    返回值：true为无异常，false为异常
  */
  bool get_electricity_data(int deviceInd,int canInd,int32_t *data);

  /*获取电机错误状态
    参数：
      deviceInd：can设备号
      canInd：can通道号
    返回值：为电机错误
      0：无错误
      1：软件错误
      2：过压
      4：欠压
      16：启动错误
  */
  int get_elektrische_Maschinen_status(int deviceInd,int canInd);

  /*将当前角度值设为0点*/
  void set_currentangle_to_zeropoints(int deviceInd,int canInd,string filename);

  /*清除电机错误
  参数：
      deviceInd：can设备号
      canInd：can通道号*/
  void clear_elc_error(int deviceInd,int canInd);

  void parse_data(uint8_t *values, uint16_t *reconstructedValues);
  int uart485_communication(char *device_485_name);

  int set_inputvalue(int value);
  void *exit_drag_teach(void *arg);

  // 拖动示教读取文件内容函数：线性插值生成插值点
  std::vector<std::vector<float>> drag_interpolate(const std::vector<float>& start, const std::vector<float>& end, int num_points);
  /*
    拖动示教读取文件内容
    参数：
      filename：文件名
      steps：插值步数
    返回值：
      0：读取成功
      1：读取失败
  */
  int drag_read_data(int deviceInd,int canInd,string filename,int steps);

  std::vector<std::array<float, 6>> linearInterpPos(const float *A, const float *B, int steps);
  void ACTmove_path(const std::vector<std::array<float, 6>> &joint_path, int deviceInd, int canInd,int coefficient);
  bool line_move(const float A[6], const float B[6], int deviceInd, int canInd, int steps,int coefficient);
  /*
  参数：
    A_pos[6]:起点
    B_pos[6]:终点
    deviceInd：can设备号
    canInd：can通道号
    steps:插值步数
    coefficient:速度系数，值越大速度越慢
  */
  bool pos_move_linear(const float A_pos[6], const float B_pos[6], int deviceInd, int canInd, int steps,int coefficient);
} // 添加extern "C"
#endif