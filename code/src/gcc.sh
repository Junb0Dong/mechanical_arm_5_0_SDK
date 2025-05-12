#!/bin/bash
export CPLUS_INCLUDE_PATH=~/mechanical_arm_5_0_SDK/code/include:$CPLUS_INCLUDE_PATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/mechanical_arm_5_0_SDK/code/include/can
g++ main.cpp  -L./include -lmylibti5 -L./include/can -I/usr/include/python3.10 -lpython3.10 -lmylibscan -lcontrolcan -lspdlog -lfmt -ludev -o move_sov
g++ keyboard_controll.cpp  -L./include -lmylibti5 -L./include/can -I/usr/include/python3.10 -lpython3.10 -lmylibscan -lcontrolcan -lspdlog -lfmt -ludev -o keyboard_controll
g++ server_main.cpp robot_server.cpp -L./include -lmylibti5 -L./include/can -I/usr/include/python3.10 -lpython3.10 -lmylibscan -lcontrolcan -lspdlog -lfmt -ludev -o robot_server
