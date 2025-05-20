#include "socket_server.h"

RobotCommandReceiver::RobotCommandReceiver(const std::string& host, int port)
    : host_(host), port_(port), server_fd_(-1), client_socket_(-1) {}

RobotCommandReceiver::~RobotCommandReceiver() {
    if (client_socket_ != -1) {
        close(client_socket_);
    }
    if (server_fd_ != -1) {
        close(server_fd_);
    }
}

bool RobotCommandReceiver::start_server() {
    // 创建套接字
    if ((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        std::cerr << "Socket creation failed" << std::endl;
        return false;
    }

    // 设置套接字选项
    int opt = 1;
    if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        std::cerr << "Setsockopt failed" << std::endl;
        return false;
    }

    // 配置服务器地址
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(port_);
    if (host_ == "0.0.0.0") {
        address.sin_addr.s_addr = INADDR_ANY;
    } else {
        if (inet_pton(AF_INET, host_.c_str(), &address.sin_addr) <= 0) {
            std::cerr << "Invalid address" << std::endl;
            return false;
        }
    }

    // 绑定
    if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        return false;
    }

    // 监听
    if (listen(server_fd_, 3) < 0) {
        std::cerr << "Listen failed" << std::endl;
        return false;
    }

    std::cout << "Server started on " << host_ << ":" << port_ << std::endl;
    return true;
}

int RobotCommandReceiver::accept_client() {
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    client_socket_ = accept(server_fd_, (struct sockaddr*)&address, (socklen_t*)&addrlen);
    if (client_socket_ < 0) {
        std::cerr << "Accept failed" << std::endl;
        return -1;
    }
    std::cout << "Client connected" << std::endl;
    return client_socket_;
}

json RobotCommandReceiver::receive_command() {
    char buffer[1024] = {0};
    int valread = read(client_socket_, buffer, 1024);
    if (valread <= 0) {
        std::cerr << "Client disconnected or read error" << std::endl;
        return json({});
    }

    try {
        json command = json::parse(buffer);
        std::cout << "Received command: " << command.dump(2) << std::endl;
        return command;
    } catch (const json::exception& e) {
        std::cerr << "JSON parse error: " << e.what() << std::endl;
        return json({{"status", "error"}, {"message", "Invalid JSON"}});
    }
}

int RobotCommandReceiver::get_client_socket() const {
    return client_socket_;
}

RobotStateSender::RobotStateSender(int client_socket) : client_socket_(client_socket) {}

void RobotStateSender::send_state(const json& state) {
    std::string state_str = state.dump();
    if (send(client_socket_, state_str.c_str(), state_str.length(), 0) < 0) {
        std::cerr << "Send failed" << std::endl;
    } else {
        std::cout << "Sent state: " << state_str << std::endl;
    }
}

void signalHandler(int signum)
{
    cout << "Interrupt signal (" << signum << ") received.\n";
    brake(0,0);
    cout << "stop!!" << endl;
    
    Exit_Can();
    exit(signum);
}

// 欧拉角转四元数函数 - 输入为vector [x,y,z,roll,pitch,yaw]
std::vector<float> eulerToQuaternion(const std::vector<float>& pose) {
    if (pose.size() != 6) {
        throw std::invalid_argument("输入向量必须包含6个元素: [x,y,z,roll,pitch,yaw]");
    }

    float x = pose[0];
    float y = pose[1];
    float z = pose[2];
    float roll = pose[3];  // 绕X轴旋转
    float pitch = pose[4]; // 绕Y轴旋转
    float yaw = pose[5];   // 绕Z轴旋转

    // 角度转弧度
    float rollRad = roll * M_PI / 180.0;
    float pitchRad = pitch * M_PI / 180.0;
    float yawRad = yaw * M_PI / 180.0;

    // 计算四元数分量
    float cr = cos(rollRad * 0.5);
    float sr = sin(rollRad * 0.5);
    float cp = cos(pitchRad * 0.5);
    float sp = sin(pitchRad * 0.5);
    float cy = cos(yawRad * 0.5);
    float sy = sin(yawRad * 0.5);

    float qw = cr * cp * cy + sr * sp * sy;
    float qx = sr * cp * cy - cr * sp * sy;
    float qy = cr * sp * cy + sr * cp * sy;
    float qz = cr * cp * sy - sr * sp * cy;

    return {x, y, z, qw, qx, qy, qz};
}
