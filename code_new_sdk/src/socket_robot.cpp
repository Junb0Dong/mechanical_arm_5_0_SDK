// build command: g++ -std=c++11 -o robot_server server_main.cpp robot_server.cpp

// bais: [171.389, -364.496, 420.007, -0.878895, 1.4993, 0.631517]

#include "socket_server.h"

using namespace std;

string filename;
char device[] = "/dev/ttyUSB0";

int main()
{
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

    // 连接到设备并enable brake
    signal(SIGINT, signalHandler);
    
    Start_Can();
    cout << "初始化CAN成功" << endl;

    // // get current pose
    // float arr[6];
    // current_pose(arr);
    // std::cout << "current pose: [";
    // for (int i = 0; i < 6; i++)
    // {
    //     arr[i] = TH.pos[i];
    //     std::cout << arr[i] << (i < 5 ? ", " : "");
    // }
    // std::cout << "]" << std::endl;

    // 机械臂的初始位置
    
    TH.pos[0] = 171.389;
    TH.pos[1] = -364.496;
    TH.pos[2] = 420.007;
    TH.pos[3] = -0.878895;
    TH.pos[4] = 1.4993;
    TH.pos[5] = 0.631517;
    bool success = move_to_pos(0, 0);
    // 初始化接收类
    // RobotCommandReceiver receiver("127.0.0.1", 9000);
    RobotCommandReceiver receiver("10.20.55.106", 9000);
    // 启动服务器
    if (!receiver.start_server())
    {
        return 1;
    }

    // 接受客户端连接
    int client_socket = receiver.accept_client(); // 接受客户端连接并返回socket描述符
    if (client_socket < 0)
    {
        return 1;
    }

    // 初始化发送类
    RobotStateSender sender(client_socket); // 使用接收到的socket描述符初始化发送类

    // 主循环
    while (true)
    {
        // 接收命令
        json command = receiver.receive_command();
        if (command.is_null() || (command.contains("status") && command["status"] == "error"))
        {
            break; // 客户端断开或错误
        }

        // 处理命令
        if (command.contains("command") && command["command"] == "set_target_pose")
        {
            try
            {
                auto pose = command["pose"].get<std::vector<double>>();
                std::cout << "Target pose: [";
                for (size_t i = 0; i < pose.size(); ++i)
                {
                    std::cout << pose[i] << (i < pose.size() - 1 ? ", " : "");
                    TH.pos[i] = pose[i]; // 更新目标位置
                }
                std::cout << "]" << std::endl;

                success = move_to_pos(0, 0); // 调用move_to_pos函数
                show_value("TH.j= ", TH.j);
                show_value("pos:", TH.pos);

                // 发送响应
                json response = {
                    {"status", success ? "success" : "error"},
                    {"message", success ? "moveJ executed" : "moveJ failed"}
                };
                sender.send_state(response);
            }
            catch (const json::exception &e)
            {
                json error_response = {
                    {"status", "error"},
                    {"message", "Invalid pose data"}};
                sender.send_state(error_response);
            }
        }
        else if (command.contains("command") && command["command"] == "moveJ")
        {
            // 处理moveJ命令
            try {
                // 解析 moveJ 参数
                auto q = command["q"].get<std::vector<double>>();

                // 验证关节数
                if (q.size() != 6) {
                    json error_response = {
                        {"status", "error"},
                        {"message", "moveJ requires exactly 6 joint positions"}
                    };
                    sender.send_state(error_response);
                    continue;
                }

                // 调用 moveJ API
                for (size_t i = 0; i < q.size(); ++i)
                {
                    TH.j[i] = q[i]; // 更新目标位置
                }
                success = move_to_joint(0, 0);
                show_value("TH.j= ", TH.j);
                show_value("pos:", TH.pos);
                std::cout << "]" << std::endl;

                json response = {
                    {"status", success ? "success" : "error"},
                    {"message", success ? "moveJ executed" : "moveJ failed"}
                };
                sender.send_state(response);
            } catch (const json::exception& e) {
                json error_response = {
                    {"status", "error"},
                    {"message", std::string("Invalid moveJ data: ") + e.what()}
                };
                sender.send_state(error_response);
            }
        }
        else
        {
            json error_response = {
                {"status", "error"},
                {"message", "Unknown command"}};
            sender.send_state(error_response);
        }
    }

    return 0;
}