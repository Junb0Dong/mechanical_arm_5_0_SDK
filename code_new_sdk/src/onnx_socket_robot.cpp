/*

===== Model Inputs =====
Input 0:
  Name:    obs
  Data Type: float
  Shape:   [1, 19]

===== Model Outputs =====
Output 0:
  Name:    actions
  Data Type: float
  Shape:   [1, 6]

*/

#include "socket_server.h"

#include <onnxruntime_cxx_api.h>
#include <chrono>  // 测量推理时间

using namespace std::chrono;

string filename;
char device[] = "/dev/ttyUSB0";

int main(int argc, char* argv[]) {
    
    // 初始化can总线
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

    string robot_ip = "10.20.55.106";

    // 连接到设备并enable brake
    signal(SIGINT, signalHandler);
    
    Start_Can();
    cout << "初始化CAN成功" << endl;

    // 加载onnx模型
    const std::string model_path = "../policy/policy.onnx"; // 替换为你的模型路径

    // 初始化onnx环境和会话
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ModelTester");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    Ort::Session session(env, model_path.c_str(), session_options);

    // ===== 手动配置输入信息（需根据你的模型修改）=====
    const std::vector<const char*> input_names = {"obs"}; // 替换为你的输入名称
    const std::vector<std::vector<int64_t>> input_shapes = {{1, 19}}; // 替换为你的输入形状
    const std::vector<const char*> output_names = {"actions"}; // 替换为你的输出名称

    // observation
    std::vector<float> current_joint_angle(6, 0.0);
    std::vector<float> target_quaternion(7, 0.0); // 目标四元数
    std::vector<float> last_actions = {0.209966, -0.721632, 0.905526, 0.0, 0.360752, 1.89849e-06}; //TODO: 需要根据初始位置更改
    // onnx input data
    std::vector<float> merged_obs;
    std::vector<Ort::Value> input_tensors;

    // 机械臂的初始位置
    TH.pos[0] = 78.82;
    TH.pos[1] = -366.26;
    TH.pos[2] = 242.61;
    TH.pos[3] = 0.21;
    TH.pos[4] = -0.03;
    TH.pos[5] = 1.99;
    std::vector<float> init_pose = {78.0628, -366.38, 240.631, 0.209908, -0.000148732, 1.99512}; //TODO: 需要根据初始位置更改
    // bool success = move_to_pos(0, 0);
    for (size_t i = 0; i < 6; i++)
    {
        TH.j[i] = last_actions[i];
    }
    bool success = move_to_joint(0, 0);

    // 初始化接收类
    RobotCommandReceiver receiver(robot_ip, 9000);

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
                // get pose from command
                auto pose = command["pose"].get<std::vector<float>>();
                // pose 需要转换成四元数
                target_quaternion = eulerToQuaternion(pose);
               
                current_angle(current_joint_angle.data(), 0, 0);
                
                // onnx推理
                // 合并所有观测数据到一个输入向量
                
                merged_obs = current_joint_angle;
                merged_obs.insert(merged_obs.end(), target_quaternion.begin(), target_quaternion.end());
                merged_obs.insert(merged_obs.end(), last_actions.begin(), last_actions.end());

                // 验证输入尺寸是否符合模型期望
                if (merged_obs.size() != static_cast<size_t>(input_shapes[0][1]))   // 显式转换为无符号  假设输入形状为 [1, obs_dim]
                { 
                    throw std::runtime_error("Error: 观测数据维度不匹配! 期望: " + 
                            std::to_string(input_shapes[0][1]) + 
                            ", 实际: " + std::to_string(merged_obs.size()));
                }

                // 创建输入张量（注意形状参数应为 {1, obs_dim}）
                Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
                input_tensors.push_back(Ort::Value::CreateTensor<float>(
                    memory_info, 
                    merged_obs.data(),       // 直接使用merged_obs的数据
                    merged_obs.size(),       // 数据大小
                    input_shapes[0].data(),  // 形状: [1, obs_dim]
                    input_shapes[0].size()   // 形状维度: 2
                ));

                // 推理代码保持不变...
                std::vector<Ort::Value> output_tensors = session.Run(
                Ort::RunOptions{nullptr},
                input_names.data(),
                input_tensors.data(), // 假设input_tensors是std::vector<Ort::Value>
                input_names.size(),
                output_names.data(),
                output_names.size()
                );

                const float* action_joint_angle = output_tensors[0].GetTensorMutableData<float>();
                std::vector<float> joint_angles(action_joint_angle, action_joint_angle + 6);
                last_actions = joint_angles;
                for (size_t i = 0; i < joint_angles.size(); i++) // 变量i改为size_t
                {
                    TH.j[i] = joint_angles[i];
                }
                move_to_joint(0, 0); // 调用move_to_joint函数
                
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
