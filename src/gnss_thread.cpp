#include "robot_state.h" // Donne acces a l'etat GNSS, au socket et a la route partages.
#include "thread_functions.h" // Rend visibles les declarations des fonctions GNSS.

#include <cmath> // Fournit atan2 pour calculer le cap GNSS.
#include <cstdio> // Fournit snprintf pour creer le message JSON.
#include <cstdlib> // Fournit atof et atoi pour convertir les champs numeriques.
#include <cstring> // Fournit les fonctions de manipulation des chaines C.
#include <iostream> // Permet d'afficher les messages de diagnostic GNSS.
#include <string> // Fournit le type texte des champs NMEA.
#include <unistd.h> // Fournit les fonctions de lecture et de temporisation serie.
#include <sys/socket.h> // Fournit send pour transmettre les trames au socket NTRIP.
#include <libwebsockets.h> // Fournit les fonctions d'envoi et de planification WebSocket.


void parse_nmea_gga(char *line) {                                                           // Analyse une trame GGA et actualise l'etat de position du robot.

    size_t len_line = strlen(line);                                                         // Calcule la longueur de la trame recue.
    while (len_line > 0 && (line[len_line - 1] == '\r' || line[len_line - 1] == '\n')) {    // Repere les retours a la ligne finaux.
        line[len_line - 1] = '\0';                                                          // Remplace le caractere de fin par le terminateur de chaine.
        len_line--;                                                                         // Met a jour la longueur apres suppression du caractere final.
    }

    if (strstr(line, "$GNGGA") == nullptr && strstr(line, "$GPGGA") == nullptr) return;     // Ignore les trames qui ne sont pas des messages GGA.


    char working_line[512];                                     // Reserve une copie modifiable de la trame.
    strncpy(working_line, line, sizeof(working_line) - 1);      // Copie la trame sans depasser la capacite disponible.
    working_line[sizeof(working_line) - 1] = '\0';              // Garantit que la copie se termine par un caractere nul.

    std::vector<std::string> tokens;    // Conserve les champs NMEA decoupes.
    char *current = working_line;       // Pointe sur le champ en cours de decoupage.
    char *next = nullptr;               // Recevra l'adresse de la prochaine virgule.

    while (current != nullptr) {                        // Parcourt les champs de la trame jusqu'au dernier.
        next = strchr(current, ',');                    // Cherche le separateur suivant dans la copie.
        if (next != nullptr) {                          // Traite un champ suivi d'une virgule.
            *next = '\0';                               // Termine le champ courant a l'emplacement de la virgule.
            tokens.push_back(std::string(current));     // Ajoute le champ courant au tableau de resultats.
            current = next + 1;                         // Positionne le pointeur sur le debut du champ suivant.
        } else {                                        // Traite le dernier champ, qui n'a pas de virgule apres lui.
            tokens.push_back(std::string(current));     // Ajoute le dernier champ au tableau.
            current = nullptr;
        }
    }


    if (tokens.size() >= 7) {                   // Verifie que les champs necessaires sont presents (latitude, direction N/S, longitude, direction E/W et mode de qualite).
        std::string raw_lat_str = tokens[2];    // Recupere la latitude au format NMEA DDMM.MMMM.
        std::string lat_dir_str = tokens[3];    // Recupere l'indicateur Nord ou Sud.
        std::string raw_lng_str = tokens[4];    // Recupere la longitude au format NMEA DDDMM.MMMM.
        std::string lng_dir_str = tokens[5];    // Recupere l'indicateur Est ou Ouest.
        std::string mode_str = tokens[6];       // Recupere le mode de qualite de la solution GNSS.

        if (!mode_str.empty()) {                // Evite de convertir un champ de qualite vide.
            int mode = atoi(mode_str.c_str());  // Convertit le mode de qualite en entier.
            current_gnss_mode = mode;           // Met a jour le mode GNSS partage avec le controle moteur.

            // Affiche la trame et son niveau de qualite pour le diagnostic.
            std::cout << "[UART-GGA] " << line << std::endl;                    // affiche le contenu de la trame GGA.
            std::cout << "[GNSS] GGA recue, qualite=" << mode << std::endl;     // affiche son mode de qualite.


            if (mode == 4 || mode == 5) {   // Seules les solutions RTK Fix (4) et RTK Float (5) mettent a jour la position visible.

                if (!raw_lat_str.empty() && !raw_lng_str.empty()) {     // Verifie que latitude et longitude sont presentes.
                    double raw_lat = atof(raw_lat_str.c_str());         // Convertit la latitude NMEA en valeur numerique.
                    double raw_lng = atof(raw_lng_str.c_str());         // Convertit la longitude NMEA en valeur numerique.

                    // Convertit les coordonnees NMEA en degres decimaux attendus par Leaflet.
                    double lat = (int)(raw_lat / 100) + (raw_lat - ((int)(raw_lat / 100) * 100)) / 60.0; // Convertit la latitude DDMM.MMMM en degres decimaux.
                    double lng = (int)(raw_lng / 100) + (raw_lng - ((int)(raw_lng / 100) * 100)) / 60.0; // Convertit la longitude DDDMM.MMMM en degres decimaux.

                    if (lat_dir_str == "S") lat = -lat; // Rend negative une latitude situee au sud de l'equateur.
                    if (lng_dir_str == "W") lng = -lng; // Rend negative une longitude situee a l'ouest du meridien.

                    route_mutex.lock();                 // Protege les mises a jour de position partagees avec les autres threads.
                    if (current_position.lat != 0.0) {  // Calcule le cap si une position precedente est deja connue.
                        current_heading = atan2(lng - current_position.lng, lat - current_position.lat) * 180.0 / M_PI; // Calcule l'angle du deplacement en degres.
                    }
                    current_position.lat = lat;     // Enregistre la nouvelle latitude.
                    current_position.lng = lng;     // Enregistre la nouvelle longitude.
                    route_mutex.unlock();           // Libere le verrou apres la mise a jour de l'etat GNSS.

                    std::cout << "[GNSS] Position parsee lat=" << lat << " lon=" << lng << " mode=" << mode << "; envoi immediat WebSocket" << std::endl; // affiche les coordonnees converties.

                    if (web_client != nullptr) {                            // Envoie une mise a jour seulement si un navigateur est connecte.
                        char json_msg[512];                                 // Reserve un tampon pour le message JSON de position.
                        int json_len = snprintf(json_msg, sizeof(json_msg), // Formate le message dans le tampon sans depassement.
                            "{\"type\":\"gnss_update\",\"lat\":%.7f,\"lng\":%.7f,\"mode\":%d,\"heading\":%.1f,\"altitude\":120.0,\"remaining_cp\":%d}", // Definit les champs transmis a l'interface.
                            lat, lng, mode, current_heading, (int)checkpoints.size()); // Insere la position, le mode, le cap et le nombre de points restants.

                        unsigned char *buf = (unsigned char*)malloc(LWS_PRE + json_len);        // Alloue l'espace de transmission requis par libwebsockets.
                        if (buf != nullptr) {                                                   // Continue seulement si l'allocation a reussi.
                            memcpy(&buf[LWS_PRE], json_msg, json_len);                          // Copie le JSON apres la zone de reserve LWS_PRE.
                            lws_write(web_client, &buf[LWS_PRE], json_len, LWS_WRITE_TEXT);     // Transmet le message JSON comme texte WebSocket.
                            free(buf);                                                          // Libere le tampon temporaire apres l'envoi.

                            lws_callback_on_writable(web_client);   // Planifie le prochain rappel d'ecriture WebSocket.
                        }
                    }
                }
            }
        }
    }
}


void gnss_io_thread() {     // Configure le recepteur puis lit ses trames serie en continu.
    char line_buffer[512];  // Accumule les caracteres jusqu'a la fin d'une ligne NMEA.
    int idx = 0;            // Indique la position d'ecriture dans le tampon courant.

    const char *cmd_unlog = "unlog\r\n";            // Prepare la commande qui arrete les journaux existants du recepteur.
    const char *cmd_rover = "mode rover\r\n";       // Prepare la commande qui configure le recepteur en mode rover.
    const char *cmd_gga = "gpgga com1 1\r\n";       // Prepare l'activation d'une trame GGA sur le port serie.
    const char *cmd_sat1 = "rtcm1033 com2\r\n";     // Prepare l'activation du message de correction RTCM 1033.
    const char *cmd_sat2 = "rtcm1006 com2\r\n";     // Prepare l'activation du message de correction RTCM 1006.
    const char *cmd_sat3 = "rtcm1074 com2\r\n";     // Prepare l'activation du message de correction RTCM 1074.
    const char *cmd_sat4 = "rtcm1124 com2\r\n";     // Prepare l'activation du message de correction RTCM 1124.
    const char *cmd_sat5 = "rtcm1082 com2\r\n";     // Prepare l'activation du message de correction RTCM 1082.
    const char *cmd_sat6 = "rtcm1094 com2\r\n";     // Prepare l'activation du message de correction RTCM 1094.
    const char *cmd_saveconfig = "saveconfig\r\n";  // Prepare la commande facultative de sauvegarde de la configuration.


    if (write(serial_fd, cmd_unlog, strlen(cmd_unlog)) < 0)             // Envoie la commande qui desactive les sorties precedentes.
        std::cerr << "[UART] Echec envoi commande unlog" << std::endl;
    else
        std::cerr << "[UART] Commande unlog envoyee" << std::endl;      // Confirme l'envoi de la commande unlog.
    usleep(150000);

    if (write(serial_fd, cmd_rover, strlen(cmd_rover)) < 0)             // Envoie la configuration du mode rover.
        std::cerr << "[UART] Echec envoi commande rover" << std::endl;
    else
        std::cerr << "[UART] Commande rover envoyee" << std::endl;      // Confirme l'envoi de la commande rover.
    usleep(150000);

    if (write(serial_fd, cmd_gga, strlen(cmd_gga)) < 0)                 // Active l'emission des trames GGA sur le port serie.
        std::cerr << "[UART] Echec envoi commande gngga 1" << std::endl;
    else
        std::cerr << "[UART] Commande gngga 1 envoyee" << std::endl;    // Confirme l'activation de GGA.
    usleep(150000);

    if (write(serial_fd, cmd_sat1, strlen(cmd_sat1)) < 0)               // Active le message RTCM 1033.
        std::cerr << "[UART] Echec envoi commande rtcm1033" << std::endl;
    else
        std::cerr << "[UART] Commande rtcm1033 envoyee" << std::endl;   // Confirme l'activation de RTCM 1033.
    usleep(150000);

    if (write(serial_fd, cmd_sat2, strlen(cmd_sat2)) < 0)               // Active le message RTCM 1006.
        std::cerr << "[UART] Echec envoi commande rtcm1006" << std::endl;
    else
        std::cerr << "[UART] Commande rtcm1006 envoyee" << std::endl;   // Confirme l'activation de RTCM 1006.
    usleep(150000);

    if (write(serial_fd, cmd_sat3, strlen(cmd_sat3)) < 0)               // Active le message RTCM 1074.
        std::cerr << "[UART] Echec envoi commande rtcm1074" << std::endl;
    else
        std::cerr << "[UART] Commande rtcm1074 envoyee" << std::endl;   // Confirme l'activation de RTCM 1074.
    usleep(150000);

    if (write(serial_fd, cmd_sat4, strlen(cmd_sat4)) < 0)               // Active le message RTCM 1124.
        std::cerr << "[UART] Echec envoi commande rtcm1124" << std::endl;
    else
        std::cerr << "[UART] Commande rtcm1124 envoyee" << std::endl;   // Confirme l'activation de RTCM 1124.
    usleep(150000);

    if (write(serial_fd, cmd_sat5, strlen(cmd_sat5)) < 0)               // Active le message RTCM 1082.
        std::cerr << "[UART] Echec envoi commande rtcm1082" << std::endl;
    else
        std::cerr << "[UART] Commande rtcm1082 envoyee" << std::endl;   // Confirme l'activation de RTCM 1082.
    usleep(150000);

    if (write(serial_fd, cmd_sat6, strlen(cmd_sat6)) < 0)               // Active le message RTCM 1094.
        std::cerr << "[UART] Echec envoi commande rtcm1094" << std::endl;
    else
        std::cerr << "[UART] Commande rtcm1094 envoyee" << std::endl;   // Confirme l'activation de RTCM 1094.
    usleep(150000);

/*
    if (write(serial_fd, cmd_saveconfig, strlen(cmd_saveconfig)) < 0)   // Envoi la commande de sauvegarde au recepteur.
        std::cerr << "[UART] Echec envoi commande saveconfig" << std::endl;
    else
        std::cerr << "[UART] Commande saveconfig envoyee" << std::endl; // Confirmerait l'envoi de la commande.
    usleep(150000);
*/


    std::cerr << "[UART] Commandes UM982 envoyees; attente des trames GGA..." << std::endl; // Signale que le recepteur est configure et que la lecture commence.


    while (true) { // Lit le flux serie jusqu'a l'arret du processus.
        char c;                                         // Recoit le prochain caractere disponible sur le port serie.
        if (read(serial_fd, &c, 1) > 0) {               // Lit un caractere lorsque le recepteur en a transmis un.
            if (c != '\r' && c != '\n' && idx < 510) {  // Accumule les caracteres sauf les fins de ligne et les depassements.
                line_buffer[idx++] = c;                 // Ajoute le caractere au tampon et avance sa position.
            } 
            else if (c == '\n') {                       // Traite le tampon lorsque le caractere de fin de ligne est recu.
                line_buffer[idx] = '\0';                // Termine la ligne accumulee comme une chaine C.

                if (idx > 10) {                         // Ignore les lignes trop courtes pour constituer une trame utile.
                    if (strstr(line_buffer, "$GNGGA") || strstr(line_buffer, "$GPGGA")) {   // Detecte les deux variantes de trame GGA.
                        std::cerr << "[UART-GGA] " << line_buffer << std::endl;             // Affiche la trame GNSS recue.
                    }
                    if ((strstr(line_buffer, "$GNGGA") || strstr(line_buffer, "$GPGGA")) && ntrip_sock != -1) { // Transmet GGA au serveur NTRIP si un socket existe.
                        char full_line[512];                                                                    // Reserve un tampon pour ajouter les fins de ligne NMEA.
                        snprintf(full_line, sizeof(full_line), "%s\r\n", line_buffer);                          // Reconstitue la trame avec ses separateurs CRLF.
                        send(ntrip_sock, full_line, strlen(full_line), 0);                                      // Envoie la trame au socket de corrections.
                    }
                    parse_nmea_gga(line_buffer);        // Analyse la ligne pour mettre a jour la position du robot.
                }
                idx = 0;                                // Reinitialise le tampon pour recevoir la prochaine ligne.
            }
        }
        usleep(500);
    }
}
