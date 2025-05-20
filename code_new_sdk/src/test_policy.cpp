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


#include <iostream>
#include <vector>
#include <string>
#include <onnxruntime_cxx_api.h>
#include <chrono>  // 测量推理时间

using namespace std::chrono;

int main(int argc, char* argv[]) {
    // 模型路径
    const std::string model_path = "../policy/policy.onnx"; // 替换为你的模型路径

    // 初始化环境和会话
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ModelTester");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    Ort::Session session(env, model_path.c_str(), session_options);

    // ===== 手动配置输入信息（需根据你的模型修改）=====
    std::vector<std::string> input_names = {"obs"}; // 替换为你的输入名称
    std::vector<std::vector<int64_t>> input_shapes = {{1, 19}}; // 替换为你的输入形状
    std::vector<std::string> output_names = {"actions"}; // 替换为你的输出名称

    // string 转换为 const char*
    std::vector<const char*> input_name_ptrs;
    for (const auto& name : input_names) {
        input_name_ptrs.push_back(name.c_str());
    }

    std::vector<const char*> output_name_ptrs;
    for (const auto& name : output_names) {
        output_name_ptrs.push_back(name.c_str());
    }

    // 准备输入数据（示例：全1数据）
    std::vector<std::vector<float>> input_values;
    for (const auto& shape : input_shapes) {
        size_t element_count = 1;
        for (int64_t dim : shape) element_count *= dim;
        input_values.emplace_back(element_count, 1.0f); // 全1数据
    }

    // 创建输入张量
    std::vector<Ort::Value> input_tensors;
    for (size_t i = 0; i < input_names.size(); i++) {
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, input_values[i].data(), input_values[i].size(), 
            input_shapes[i].data(), input_shapes[i].size()
        ));
    }
    
    // ===== 新增：推理时间测量 =====
    // 记录开始时间（使用高精度时钟）
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::vector<Ort::Value> output_tensors = session.Run(
    Ort::RunOptions{nullptr},
    input_name_ptrs.data(),
    input_tensors.data(), // 假设input_tensors是std::vector<Ort::Value>
    input_names.size(),
    output_name_ptrs.data(),
    output_names.size()
    );
    
    // 记录结束时间
    auto end_time = std::chrono::high_resolution_clock::now();
    // 计算时间差（单位：毫秒）
    auto duration = duration_cast<milliseconds>(end_time - start_time);
    std::cout << "Inference time: " << duration.count() << " ms" << std::endl;

    // 处理输出（动态判断数据类型）
    for (size_t i = 0; i < output_tensors.size(); i++) {
        auto& output_tensor = output_tensors[i];
        auto tensor_info = output_tensor.GetTensorTypeAndShapeInfo();
        ONNXTensorElementDataType element_type = tensor_info.GetElementType();
        std::vector<int64_t> output_shape = tensor_info.GetShape();

        std::cout << "Output " << i << " Shape: [";
        for (auto dim : output_shape) std::cout << dim << " ";
        std::cout << "]" << std::endl;

        const float* output_data = output_tensor.GetTensorMutableData<float>();
        std::cout << "Output Data: ";
        for (size_t j = 0; j < 6; j++) {
            std::cout << output_data[j] << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
