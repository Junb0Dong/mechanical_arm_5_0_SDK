#!/bin/bash
export CPLUS_INCLUDE_PATH=~/mechanical_arm_5_0_SDK/code_new_sdk/include:$CPLUS_INCLUDE_PATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/mechanical_arm_5_0_SDK/code_new_sdk/include
g++ main.cpp -L../include -I/usr/include/python3.10 -lpython3.10  -I /usr/include/eigen3 -lcontrolcan -lmylibti5 -o move_sov
g++ move_origin.cpp -L../include -I/usr/include/python3.10 -lpython3.10  -I /usr/include/eigen3 -lcontrolcan -lmylibti5 -o move_origin
g++ server_main.cpp robot_server.cpp -L../include -I/usr/include/python3.10 -lpython3.10  -I /usr/include/eigen3 -lcontrolcan -lmylibti5 -o robot_server
