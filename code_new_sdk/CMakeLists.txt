cmake_minimum_required(VERSION 3.14)
project(ONNXRT_Training_Demo)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# 添加头文件搜索路径
include_directories(
    ${CMAKE_SOURCE_DIR}/include           # 项目头文件
    /usr/include/python3.10              # Python头文件
    /usr/include/eigen3                  # Eigen3头文件
)

# 定义通用库列表
set(COMMON_LIBS 
    python3.10
    controlcan
    mylibti5
)

# 手动查找ONNX Runtime库
find_library(ONNXRT_LIB 
    NAMES onnxruntime
    HINTS /usr/local/lib
    REQUIRED)

# 添加ONNX Runtime头文件目录
include_directories(/usr/local/include/onnxruntime)

message(STATUS "ONNX Runtime library: ${ONNXRT_LIB}")

# 添加可执行文件并链接库
add_executable(move_sov src/main.cpp)
add_executable(move_origin src/move_origin.cpp)
add_executable(robot_server src/server_main.cpp src/robot_server.cpp)
add_executable(read_onnx_demo src/read_onnx.cpp)
add_executable(test_policy src/test_policy.cpp)

# 链接通用库
target_link_libraries(move_sov ${COMMON_LIBS})
target_link_libraries(move_origin ${COMMON_LIBS})
target_link_libraries(robot_server ${COMMON_LIBS})

# 链接ONNX Runtime库
target_link_libraries(read_onnx_demo ${COMMON_LIBS} ${ONNXRT_LIB})
target_link_libraries(test_policy ${COMMON_LIBS} ${ONNXRT_LIB})

# 安装规则（可选）
install(TARGETS 
    move_sov 
    move_origin 
    robot_server 
    read_onnx_demo
    test_policy
    DESTINATION bin)
