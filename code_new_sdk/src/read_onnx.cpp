#include <iostream>
#include <vector>
#include <string>
#include <onnxruntime_cxx_api.h>

// 辅助函数：将ONNX数据类型转换为可读字符串
std::string getDataTypeString(ONNXTensorElementDataType type) {
    switch (type) {
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:          return "float";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:          return "uint8_t";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:           return "int8_t";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16:         return "uint16_t";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:          return "int16_t";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:          return "int32_t";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:          return "int64_t";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_STRING:         return "string";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL:           return "bool";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16:        return "float16";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:         return "double";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32:         return "uint32_t";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64:         return "uint64_t";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_COMPLEX64:      return "complex64";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_COMPLEX128:     return "complex128";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_BFLOAT16:       return "bfloat16";
        default:                                             return "unknown";
    }
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <model_path.onnx>" << std::endl;
        return 1;
    }
    std::cout << "had changed test " << argv[1] << std::endl;
    const std::string model_path = argv[1];
    
    // 初始化环境
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ModelInfoPrinter");
    
    // 初始化会话选项
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    
    // 创建会话并加载模型
    Ort::Session session(env, model_path.c_str(), session_options);
    
    // 获取输入信息
    std::cout << "===== Model Inputs =====" << std::endl;
    size_t num_inputs = session.GetInputCount();
    
    for (size_t i = 0; i < num_inputs; i++) {
        // 获取输入名称（旧版API兼容写法）
        std::vector<std::string> input_names = session.GetInputNames();
        std::string input_name = input_names[i];
        
        // 获取输入类型信息
        Ort::TypeInfo input_type_info = session.GetInputTypeInfo(i);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        
        // 获取输入形状
        std::vector<int64_t> input_dims = input_tensor_info.GetShape();
        
        // 获取输入数据类型（转换为可读字符串）
        ONNXTensorElementDataType input_type = input_tensor_info.GetElementType();
        std::string input_type_str = getDataTypeString(input_type);
        
        // 输出格式化的输入信息
        std::cout << "Input " << i << ":\n"
                  << "  Name:    " << input_name << "\n"
                  << "  Data Type: " << input_type_str << "\n"
                  << "  Shape:   [";
        for (size_t j = 0; j < input_dims.size(); j++) {
            std::cout << input_dims[j];
            if (j < input_dims.size() - 1) std::cout << ", ";
        }
        std::cout << "]\n\n";
    }
    
    // 获取输出信息
    std::cout << "===== Model Outputs =====" << std::endl;
    size_t num_outputs = session.GetOutputCount();
    
    for (size_t i = 0; i < num_outputs; i++) {
        // 获取输出名称（旧版API兼容写法）
        std::vector<std::string> output_names = session.GetOutputNames();
        std::string output_name = output_names[i];
        
        // 获取输出类型信息
        Ort::TypeInfo output_type_info = session.GetOutputTypeInfo(i);
        auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
        
        // 获取输出形状
        std::vector<int64_t> output_dims = output_tensor_info.GetShape();
        
        // 获取输出数据类型（转换为可读字符串）
        ONNXTensorElementDataType output_type = output_tensor_info.GetElementType();
        std::string output_type_str = getDataTypeString(output_type);
        
        // 输出格式化的输出信息
        std::cout << "Output " << i << ":\n"
                  << "  Name:    " << output_name << "\n"
                  << "  Data Type: " << output_type_str << "\n"
                  << "  Shape:   [";
        for (size_t j = 0; j < output_dims.size(); j++) {
            std::cout << output_dims[j];
            if (j < output_dims.size() - 1) std::cout << ", ";
        }
        std::cout << "]\n\n";
    }
    
    return 0;
}
