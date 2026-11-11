#pragma once        // Evite les inclusions multiples de ce fichier d'etat partage.

#include <mutex>    // Fournit les verrous qui protegent les donnees partagees.
#include <string>   // Fournit le type texte utilise par l'etat du robot.
#include <vector>   // Fournit le conteneur de points de passage.

struct lws;         // Declare le type libwebsockets sans inclure toute sa bibliotheque.

struct Point {      // Represente une position geographique.
    double lat;     // Stocke la latitude en degres decimaux.
    double lng;     // Stocke la longitude en degres decimaux.
}; 

extern std::vector<Point> checkpoints;  // Declare la route partagee entre les threads.
extern std::mutex route_mutex;          // Declare le verrou de la route et de l'etat GNSS.
extern struct lws *web_client;          // Declare le client WebSocket actuellement connecte.
extern std::mutex ws_message_mutex;     // Declare le verrou de la file de messages WebSocket.
extern std::string ws_pending_message;  // Declare le prochain message en attente d'envoi.
extern int serial_fd;                   // Declare le descripteur du port serie GNSS.
extern int ntrip_sock;                  // Declare le socket de corrections NTRIP, invalide s'il vaut -1.
extern Point current_position;          // Declare la derniere position GNSS connue.
extern double current_heading;          // Declare le cap calcule a partir des positions GNSS.
extern int current_gnss_mode;           // Declare le mode de qualite de la derniere position GNSS.
extern std::string manual_direction;    // Declare l'ordre de deplacement manuel actif.
