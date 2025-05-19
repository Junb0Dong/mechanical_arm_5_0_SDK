#include <string>
#include <vector>
#include <sys/socket.h>
#include <nlohmann/json.hpp>
#include <arpa/inet.h>
#include <unistd.h>


#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>

#include "Ti5CAN_Driver.h"
#include "Ti5MOVE.h"
#include "Ti5BASIC.h"
#include "Ti5LOGIC.h"
#include "communication.h"
#include "mathfunc.h"
#include "tool.h"
#include "clamping_jaw.h"
#include "drag_drop.h"
#include "socket_service.h"
#include "example.h"
#include "key_event.h"


// 定义颜色转义序列
#define RESET "\033[0m"
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */

using json = nlohmann::json;

class RobotCommandReceiver
{
public:
    RobotCommandReceiver(const std::string &host, int port); // Constructor
    ~RobotCommandReceiver();                                 // Destructor

    bool start_server();           // Start the server
    int accept_client();           // Accept a client connection
    json receive_command();        // Receive a command from the client
    int get_client_socket() const; // Get the client socket file descriptor

private:
    std::string host_;
    int port_;
    int server_fd_;
    int client_socket_;
};

class RobotStateSender
{
public:
    RobotStateSender(int client_socket); // Constructor
    void send_state(const json &state);  // Send the robot state to the client

private:
    int client_socket_;
};
void signalHandler(int signum);