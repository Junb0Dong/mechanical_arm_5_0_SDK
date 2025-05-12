#include <string>
#include <vector>
#include <sys/socket.h>
#include <nlohmann/json.hpp>

#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <cstdlib>

#include "Ti5MOVE.h"
#include "Ti5BASIC.h"
#include "Ti5LOGIC.h"
#include "communication.h"
#include "mathfunc.h"
#include "tool.h"
#include "clamping_jaw.h"
#include "drag_drop.h"

#include <csignal>

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