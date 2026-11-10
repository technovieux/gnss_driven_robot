//sudo apt update
//sudo apt install build-essential pkg-config libwebsockets-dev
//make -C src


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <thread>
#include <vector>
#include <mutex>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cstdint>
#include <cmath>
#include <libwebsockets.h>
#include <cstdlib>
#include <signal.h>

#include "robot_state.h"
#include "thread_functions.h"

#define SERIAL_PORT    "/dev/serial0"
#define BAUDRATE       B115200
#define WS_PORT        9002

std::vector<Point> checkpoints;
std::mutex route_mutex;
struct lws *web_client = nullptr;
std::mutex ws_message_mutex;
std::string ws_pending_message;
int serial_fd = -1;
int ntrip_sock = -1;

Point current_position = {0.0, 0.0};
double current_heading = 0.0;
int current_gnss_mode = 0;

std::string manual_direction = "STOP";

void signal_handler(int sig) {
    std::cerr << "\n[SYSTEM] Arrêt du robot demandé (Signal " << sig << ")..." << std::endl;
    
    if (serial_fd != -1) {
        close(serial_fd);
        std::cerr << "[UART] Port série fermé proprement." << std::endl;
    }
    
    system("pkill -f 'http.server 8080'");
    
    exit(0);
}

int init_serial(const char *portname) {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) return -1;
    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        perror("[UART] tcgetattr");
        close(fd);
        return -1;
    }
    cfmakeraw(&options);
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8 | CLOCAL | CREAD;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("[UART] tcsetattr");
        close(fd);
        return -1;
    }
    tcflush(fd, TCIFLUSH);
    std::cerr << "[UART] Port " << portname << " ouvert en 115200 8N1 (mode brut)" << std::endl;
    return fd;
}

static int callback_websocket(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len) {
    switch (reason) {
        case LWS_CALLBACK_ESTABLISHED:
            web_client = wsi;
            std::cerr << "[WS] Client navigateur connecte" << std::endl;
            break;
        case LWS_CALLBACK_CLOSED:
            if (web_client == wsi) web_client = nullptr;
            std::cerr << "[WS] Client navigateur deconnecte" << std::endl;
            break;
        case LWS_CALLBACK_SERVER_WRITEABLE: {
            std::string message;
            {
                std::lock_guard<std::mutex> lock(ws_message_mutex);
                message.swap(ws_pending_message);
            }
            if (!message.empty()) {
                std::vector<unsigned char> buf(LWS_PRE + message.size());
                memcpy(buf.data() + LWS_PRE, message.data(), message.size());
                const int written = lws_write(wsi, buf.data() + LWS_PRE, message.size(), LWS_WRITE_TEXT);
                if (written < 0) {
                    std::cerr << "[WS] Echec lws_write" << std::endl;
                    return -1;
                }
                if ((size_t)written != message.size()) {
                    std::cerr << "[WS] Avertissement: message tronque (" << written
                              << "/" << message.size() << ")" << std::endl;
                } else {
                    std::cerr << "[WS] Message envoye au navigateur: " << message << std::endl;
                }
            }
            break;
        }
        case LWS_CALLBACK_RECEIVE: {
            const std::string payload((const char *)in, len);
            if (payload.find("\"action\":\"add_checkpoint\"") != std::string::npos) {
                const char *lat_ptr = strstr(payload.c_str(), "\"lat\":");
                const char *lng_ptr = strstr(payload.c_str(), "\"lng\":");
                if (lat_ptr && lng_ptr) {
                    const double lat = atof(lat_ptr + 6);
                    const double lng = atof(lng_ptr + 6);
                    std::lock_guard<std::mutex> lock(route_mutex);
                    checkpoints.push_back({lat, lng});
                }
            } else if (payload.find("\"action\":\"clear_route\"") != std::string::npos) {
                std::lock_guard<std::mutex> lock(route_mutex);
                checkpoints.clear();
            } else if (payload.find("\"action\":\"manual_move\"") != std::string::npos) {
                const char *dir_ptr = strstr(payload.c_str(), "\"direction\":\"");
                if (dir_ptr) {
                    char dir[16] = {};
                    if (sscanf(dir_ptr + 13, "%15[^\"]", dir) == 1) {
                        std::lock_guard<std::mutex> lock(route_mutex);
                        manual_direction = std::string(dir);
                    }
                }
            }
            break;
        }
        default:
            break;
    }
    return 0;
}

static struct lws_protocols protocols[] = {
    { "robot-protocol", callback_websocket, 0, 4096, 0, nullptr, 0 },
    { nullptr, nullptr, 0, 0, 0, nullptr, 0 }
};

int main() {

    signal(SIGINT, signal_handler);

    serial_fd = init_serial(SERIAL_PORT);
    if (serial_fd == -1) {
        std::cerr << "Erreur de liaison série UART." << std::endl;
        return EXIT_FAILURE;
    }

    std::thread t_gnss(gnss_io_thread);
    std::thread t_motor(motor_control_thread);
    std::thread t_stats(system_stats_thread);
    std::thread t_web(web_server_thread);
    
    struct lws_context_creation_info ws_info;
    memset(&ws_info, 0, sizeof(ws_info));
    ws_info.port = WS_PORT;
    ws_info.protocols = protocols;

    ws_info.iface = nullptr; 

    struct lws_context *context = lws_create_context(&ws_info);
    if (context == nullptr) {
        std::cerr << "[WS] Impossible de creer le serveur WebSocket sur le port " << WS_PORT << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "[SYSTEM] Serveur WebSocket en ecoute sur le port " << WS_PORT << std::endl;
    
    while (true) {
        lws_service(context, 5);
        bool pending = false;
        {
            std::lock_guard<std::mutex> lock(ws_message_mutex);
            pending = !ws_pending_message.empty();
        }
        if (pending && web_client != nullptr) lws_callback_on_writable(web_client);
    }
    
    lws_context_destroy(context);

    system("pkill -f 'http.server 8080'");

    if (t_web.joinable()) {
        t_web.join();
    }

    t_gnss.join(); 
    t_motor.join(); 
    t_stats.join();
    
    return 0;
}