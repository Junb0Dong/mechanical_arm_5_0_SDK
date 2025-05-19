#ifndef TI5CAN_DRIVER_H
#define TI5CAN_DRIVER_H

#include <iostream>
#include <string>
#include <unistd.h>
// #include "can/controlcan.h"
#include "controlcan.h"

#define MOTOR_ENABLE 1 //使能电机指令（1字节）
#define MOTOR_STOP_MODE 2 //停止模式指令（1字节）
#define GET_MOTOR_PERSIONS  8 //获取电机当前位置指令（1字节），转化为减速机角度公式：(返回值/65536/减速比)*360
#define GET_MOTOR_ERROR     10  //获取电机错误指令（1字节）
#define CLEAR_ERROR         11 //清除电机错误指令（1字节）
#define SET_MOTOR_CURRENTS  28 //设置电机为电流模式，并设置目标电流指令（5字节）
#define SET_MOTOR_SPEED     29  //设置电机为速度模式，并设置目标速度指令（5字节），下发参数为：(目标转速（度每秒）*减速比*100)/360
#define SET_MOTOR_POSITION  30 //设置电机为位置模式，并设置目标位置指令（5字节），下发参数为：(减速机目标角度/360)*减速比*65536
#define SET_MOTOR_MAX_SPEED 36 //设置电机最大正向允许速度指令（5字节），下发参数为：(目标转速（度每秒）*减速比*100)/360
#define SET_MOTOR_MIN_SPEED 37  //设置电机最小负向允许速度指令（5字节），下发参数为：(目标转速（度每秒）*减速比*100)/360


extern int CanNum;
 /*登录并初始化can设备
    参数：无
    返回值
      true：成功
      false：失败
  */
bool Start_Can();

/*登出can设备
    参数：无
    返回值
      true：成功
      false：失败
  */
bool Exit_Can();

int32_t convertHexArrayToDecimal(const uint8_t hexArray[4]);

void toIntArray(int number, int *res, int size);

/*发送1字节指令
  参数：
    DeviceInd：can设备索引 （一个为0,2个为1）
    CANInd：can通道索引 （通道1：0,通道2：1）
    numOfActuator：发送指令的电机数量
    canIdList：电机canId列表
    commandList：指令列表
    dataList：接收数据列表
*/
void sendSimpleCanCommand(int DeviceInd, int CANInd,uint8_t numOfActuator, uint8_t *canIdList, uint8_t command,int32_t *dataList);
/*发送5字节指令
  参数：
    DeviceInd：can设备索引 （一个为0,2个为1）
    CANInd：can通道索引 （通道1：0,通道2：1）
    numOfActuator：发送指令的电机数量
    canIdList：电机canId列表
    commandList：指令列表
    dataList：接收数据列表
*/
void sendCanCommand(int DeviceInd,int CANInd,uint8_t numOfActuator, uint8_t *canIdList, uint8_t command, uint32_t *parameterList);

#endif // TEST_CAN_H
