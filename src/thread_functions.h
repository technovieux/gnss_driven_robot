#pragma once // Evite les inclusions multiples des declarations de threads.

void system_stats_thread();         // Declare le thread de collecte des statistiques systeme.
void gnss_io_thread();              // Declare le thread de lecture du recepteur GNSS.
void motor_control_thread();        // Declare le thread de commande des moteurs.
void web_server_thread();           // Declare le thread qui lance le serveur HTTP.
void parse_nmea_gga(char *line);    // Declare l'analyseur des trames NMEA GGA.
