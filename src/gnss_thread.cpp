#include "robot_state.h"
#include "thread_functions.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/socket.h>
#include <libwebsockets.h>

void parse_nmea_gga(char *line) {
    size_t len_line = strlen(line);
    while (len_line > 0 && (line[len_line - 1] == '\r' || line[len_line - 1] == '\n')) {
        line[len_line - 1] = '\0';
        len_line--;
    }

    if (strstr(line, "$GNGGA") == nullptr && strstr(line, "$GPGGA") == nullptr) return;

    char working_line[512];
    strncpy(working_line, line, sizeof(working_line) - 1);
    working_line[sizeof(working_line) - 1] = '\0';

    std::vector<std::string> tokens;
    char *current = working_line;
    char *next = nullptr;

    while (current != nullptr) {
        next = strchr(current, ',');
        if (next != nullptr) {
            *next = '\0';
            tokens.push_back(std::string(current));
            current = next + 1;
        } else {
            tokens.push_back(std::string(current));
            current = nullptr;
        }
    }

    // Une trame GGA doit posséder au moins 15 champs séparés par des virgules
    if (tokens.size() >= 7) {
        // 1 = Heure UTC | 2 = Latitude | 3 = N/S | 4 = Longitude | 5 = E/W | 6 = Mode RTK
        std::string raw_lat_str = tokens[2];
        std::string lat_dir_str = tokens[3];
        std::string raw_lng_str = tokens[4];
        std::string lng_dir_str = tokens[5];
        std::string mode_str = tokens[6];

        if (!mode_str.empty()) {
            int mode = atoi(mode_str.c_str());
            current_gnss_mode = mode;

            // Message de logs et diagnostic dans votre terminal Pi
            std::cout << "[UART-GGA] " << line << std::endl;
            std::cout << "[GNSS] GGA recue, qualite=" << mode << std::endl;

            // 🎯 FILTRAGE STRICT : On ne pousse à l'interface Leaflet que si RTK Fix (4) ou RTK Float (5)
            if (mode == 4 || mode == 5) {
                if (!raw_lat_str.empty() && !raw_lng_str.empty()) {
                    double raw_lat = atof(raw_lat_str.c_str());
                    double raw_lng = atof(raw_lng_str.c_str());

                    // Conversion mathématique du format aéronautique DDMM.MMMM en Degrés Décimaux (Leaflet)
                    double lat = (int)(raw_lat / 100) + (raw_lat - ((int)(raw_lat / 100) * 100)) / 60.0;
                    double lng = (int)(raw_lng / 100) + (raw_lng - ((int)(raw_lng / 100) * 100)) / 60.0;

                    if (lat_dir_str == "S") lat = -lat;
                    if (lng_dir_str == "W") lng = -lng;

                    route_mutex.lock();
                    if (current_position.lat != 0.0) {
                        current_heading = atan2(lng - current_position.lng, lat - current_position.lat) * 180.0 / M_PI;
                    }
                    current_position.lat = lat;
                    current_position.lng = lng;
                    route_mutex.unlock();

                    std::cout << "[GNSS] Position parsee lat=" << lat << " lon=" << lng << " mode=" << mode << "; envoi immediat WebSocket" << std::endl;

                    if (web_client != nullptr) {
                        char json_msg[512];
                        int json_len = snprintf(json_msg, sizeof(json_msg),
                            "{\"type\":\"gnss_update\",\"lat\":%.7f,\"lng\":%.7f,\"mode\":%d,\"heading\":%.1f,\"altitude\":120.0,\"remaining_cp\":%d}",
                            lat, lng, mode, current_heading, (int)checkpoints.size());

                        unsigned char *buf = (unsigned char*)malloc(LWS_PRE + json_len);
                        if (buf != nullptr) {
                            memcpy(&buf[LWS_PRE], json_msg, json_len);
                            lws_write(web_client, &buf[LWS_PRE], json_len, LWS_WRITE_TEXT);
                            free(buf);

                            lws_callback_on_writable(web_client);
                        }
                    }
                }
            }
        }
    }
}

void gnss_io_thread() {
    char line_buffer[512];
    int idx = 0;

    const char *cmd_unlog = "unlog\r\n";
    const char *cmd_rover = "mode rover\r\n";
    const char *cmd_gga = "gpgga com1 1\r\n";
    const char *cmd_sat1 = "rtcm1033 com2\r\n";
    const char *cmd_sat2 = "rtcm1006 com2\r\n";
    const char *cmd_sat3 = "rtcm1074 com2\r\n";
    const char *cmd_sat4 = "rtcm1124 com2\r\n";
    const char *cmd_sat5 = "rtcm1082 com2\r\n";
    const char *cmd_sat6 = "rtcm1094 com2\r\n";
    const char *cmd_saveconfig = "saveconfig\r\n";

    if (write(serial_fd, cmd_unlog, strlen(cmd_unlog)) < 0)
        std::cerr << "[UART] Echec envoi commande unlog" << std::endl;
    else
        std::cerr << "[UART] Commande unlog envoyee" << std::endl;
    usleep(150000);

    if (write(serial_fd, cmd_rover, strlen(cmd_rover)) < 0)
        std::cerr << "[UART] Echec envoi commande rover" << std::endl;
    else
        std::cerr << "[UART] Commande rover envoyee" << std::endl;
    usleep(150000);

    if (write(serial_fd, cmd_gga, strlen(cmd_gga)) < 0)
        std::cerr << "[UART] Echec envoi commande gngga 1" << std::endl;
    else
        std::cerr << "[UART] Commande gngga 1 envoyee" << std::endl;
    usleep(150000);

    if (write(serial_fd, cmd_sat1, strlen(cmd_sat1)) < 0)
        std::cerr << "[UART] Echec envoi commande rtcm1033" << std::endl;
    else
        std::cerr << "[UART] Commande rtcm1033 envoyee" << std::endl;
    usleep(150000);

    if (write(serial_fd, cmd_sat2, strlen(cmd_sat2)) < 0)
        std::cerr << "[UART] Echec envoi commande rtcm1006" << std::endl;
    else
        std::cerr << "[UART] Commande rtcm1006 envoyee" << std::endl;
    usleep(150000);

    if (write(serial_fd, cmd_sat3, strlen(cmd_sat3)) < 0)
        std::cerr << "[UART] Echec envoi commande rtcm1074" << std::endl;
    else
        std::cerr << "[UART] Commande rtcm1074 envoyee" << std::endl;
    usleep(150000);

    if (write(serial_fd, cmd_sat4, strlen(cmd_sat4)) < 0)
        std::cerr << "[UART] Echec envoi commande rtcm1124" << std::endl;
    else
        std::cerr << "[UART] Commande rtcm1124 envoyee" << std::endl;
    usleep(150000);

    if (write(serial_fd, cmd_sat5, strlen(cmd_sat5)) < 0)
        std::cerr << "[UART] Echec envoi commande rtcm1082" << std::endl;
    else
        std::cerr << "[UART] Commande rtcm1082 envoyee" << std::endl;
    usleep(150000);

    if (write(serial_fd, cmd_sat6, strlen(cmd_sat6)) < 0)
        std::cerr << "[UART] Echec envoi commande rtcm1094" << std::endl;
    else
        std::cerr << "[UART] Commande rtcm1094 envoyee" << std::endl;
    usleep(150000);

/*
    if (write(serial_fd, cmd_saveconfig, strlen(cmd_saveconfig)) < 0)
        std::cerr << "[UART] Echec envoi commande saveconfig" << std::endl;
    else
        std::cerr << "[UART] Commande saveconfig envoyee" << std::endl;
    usleep(150000);
*/

    std::cerr << "[UART] Commandes UM982 envoyees; attente des trames GGA..." << std::endl;

    while (true) {
        char c;
        if (read(serial_fd, &c, 1) > 0) {
            if (c != '\r' && c != '\n' && idx < 510) {
                line_buffer[idx++] = c;
            } else if (c == '\n') {
                line_buffer[idx] = '\0';
                if (idx > 10) {
                    if (strstr(line_buffer, "$GNGGA") || strstr(line_buffer, "$GPGGA")) {
                        std::cerr << "[UART-GGA] " << line_buffer << std::endl;
                    }
                    if ((strstr(line_buffer, "$GNGGA") || strstr(line_buffer, "$GPGGA")) && ntrip_sock != -1) {
                        char full_line[512];
                        snprintf(full_line, sizeof(full_line), "%s\r\n", line_buffer);
                        send(ntrip_sock, full_line, strlen(full_line), 0);
                    }
                    parse_nmea_gga(line_buffer);
                }
                idx = 0;
            }
        }
        usleep(500);
    }
}
