#pragma once

#include <mutex>
#include <string>
#include <vector>

struct lws;

struct Point {
    double lat;
    double lng;
};

extern std::vector<Point> checkpoints;
extern std::mutex route_mutex;
extern struct lws *web_client;
extern std::mutex ws_message_mutex;
extern std::string ws_pending_message;
extern int serial_fd;
extern int ntrip_sock;
extern Point current_position;
extern double current_heading;
extern int current_gnss_mode;
extern std::string manual_direction;
