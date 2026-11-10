#pragma once

void system_stats_thread();
void gnss_io_thread();
void motor_control_thread();
void web_server_thread();
void parse_nmea_gga(char *line);
