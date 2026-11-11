//sudo apt update // Met a jour la liste des paquets sur un systeme Debian/Ubuntu.
//sudo apt install build-essential pkg-config libwebsockets-dev // Installe le compilateur et la dependance WebSocket.
//make -C src // Compile le programme avec le Makefile du dossier src.

#include <iostream>             // Fournit les flux d'affichage et de journalisation.
#include <stdio.h>              // Fournit les fonctions C de formatage et de lecture.
#include <stdlib.h>             // Fournit les fonctions C de gestion du processus et de conversion.
#include <string.h>             // Fournit les fonctions de manipulation des chaines C.
#include <unistd.h>             // Fournit close et les fonctions POSIX utilisees par le port serie.
#include <fcntl.h>              // Fournit les options d'ouverture du port serie.
#include <termios.h>            // Fournit la configuration POSIX du port serie.
#include <sys/socket.h>         // Fournit les types et fonctions de sockets.
#include <netdb.h>              // Fournit les declarations reseau standard.
#include <arpa/inet.h>          // Fournit les conversions d'adresses IP.
#include <thread>               // Permet de lancer les threads du robot.
#include <vector>               // Fournit le conteneur utilise pour les messages WebSocket.
#include <mutex>                // Fournit les verrous des etats partages.
#include <string>               // Fournit le type texte de la file WebSocket.
#include <sstream>              // Fournit les flux de texte standard.
#include <fstream>              // Fournit les flux de fichiers standard.
#include <iomanip>              // Fournit les options de formatage numerique.
#include <cstdint>              // Fournit les types entiers de taille fixe.
#include <cmath>                // Fournit les fonctions mathematiques standard.
#include <libwebsockets.h>      // Fournit les declarations du serveur WebSocket.
#include <cstdlib>              // Fournit system et les autres fonctions standard de processus.
#include <signal.h>             // Fournit la gestion du signal d'interruption.

#include "robot_state.h"        // Declare l'etat de partage du robot.
#include "thread_functions.h"   // Declare les fonctions executees dans les threads.

#define SERIAL_PORT    "/dev/serial0"   // chemin du port serie du recepteur GNSS.
#define BAUDRATE       B115200          // debit de communication serie a 115200 bauds.
#define WS_PORT        9002             // port d'ecoute du serveur WebSocket.

std::vector<Point> checkpoints;         // Definit la liste partagee des points de passage.
std::mutex route_mutex;                 // Definit le verrou qui protege la route et les donnees GNSS.
struct lws *web_client = nullptr;       // Initialise le pointeur du client WebSocket a l'etat deconnecte.
std::mutex ws_message_mutex;            // Definit le verrou de la file des messages WebSocket.
std::string ws_pending_message;         // Definit le message WebSocket en attente d'envoi.
int serial_fd = -1;                     // Initialise le descripteur serie a l'etat invalide.
int ntrip_sock = -1;                    // Initialise le socket NTRIP a l'etat invalide.

Point current_position = {0.0, 0.0};    // Initialise la derniere position GNSS a l'origine.
double current_heading = 0.0;           // Initialise le cap du robot en degres.
int current_gnss_mode = 0;              // Initialise le mode GNSS avant toute mesure.

std::string manual_direction = "STOP";  // Initialise le mode manuel a l'arret.

void signal_handler(int sig) {          // Effectue l'arret demande par un signal du systeme.
    std::cerr << "\n[SYSTEM] Arrêt du robot demandé (Signal " << sig << ")..." << std::endl;    // Signale la reception du signal.
    
    if (serial_fd != -1) {              // Verifie qu'un port serie a bien ete ouvert.
        close(serial_fd);               // Ferme le descripteur du port serie.
        std::cerr << "[UART] Port série fermé proprement." << std::endl;    // Confirme la fermeture du port.
    }
    
    system("pkill -f 'http.server 8080'");  // Arrete le serveur HTTP Python lance pour l'interface.
    
    exit(0);
}

int init_serial(const char *portname) {                         // Ouvre et configure le port serie utilise par le recepteur.
    int fd = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);    // Ouvre le port en lecture-ecriture sans en faire un terminal de controle.
    if (fd == -1) return -1;                                    // Retourne une valeur invalide si l'ouverture echoue.
    struct termios options;                                     // Declare la structure des options de communication serie.
    if (tcgetattr(fd, &options) != 0) {                         // Recupere la configuration serie actuelle.
        perror("[UART] tcgetattr");                             // Affiche la cause systeme de l'echec de lecture de configuration.
        close(fd);                                              // Ferme le descripteur ouvert avant de quitter.
        return -1;                                              // Indique que l'initialisation du port a echoue.
    }                                                          
    cfmakeraw(&options);                                        // Configure le port en mode brut sans traitement de texte.
    cfsetispeed(&options, BAUDRATE);                            // Definit le debit de reception.
    cfsetospeed(&options, BAUDRATE);                            // Definit le debit d'emission.
    options.c_cflag &= ~PARENB;                                 // Desactive le controle de parite.
    options.c_cflag &= ~CSTOPB;                                 // Configure une seule bit d'arret.
    options.c_cflag &= ~CSIZE;                                  // Efface le nombre de bits de donnees precedemment configure.
    options.c_cflag |= CS8 | CLOCAL | CREAD;                    // Configure huit bits de donnees et active reception locale.
    options.c_cc[VMIN] = 0;                                     // N'exige aucun minimum de caracteres pour la lecture.
    options.c_cc[VTIME] = 1;                                    // Definit un delai de lecture de 0,1 seconde.
    if (tcsetattr(fd, TCSANOW, &options) != 0) {                // Applique immediatement la configuration du port.
        perror("[UART] tcsetattr");                             // Affiche la cause systeme de l'echec de configuration.
        close(fd);                                              // Ferme le descripteur si les options n'ont pas pu etre appliquees.
        return -1;                                              // Indique que l'initialisation du port a echoue.
    }                                                           
    tcflush(fd, TCIFLUSH);                                      // Vide les caracteres deja presents dans le tampon de reception.

    std::cerr << "[UART] Port " << portname << " ouvert en 115200 8N1 (mode brut)" << std::endl; // Confirme l'ouverture et le format du port.

    return fd; // Retourne le descripteur serie configure.
}

static int callback_websocket(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len) { // Traite les evenements du client WebSocket.
    switch (reason) {                                                                           // Selectionne le traitement selon le type de rappel libwebsockets.
        case LWS_CALLBACK_ESTABLISHED:                                                          // Traite la connexion d'un navigateur.
            web_client = wsi;                                                                   // Enregistre le client WebSocket courant.
            std::cerr << "[WS] Client navigateur connecte" << std::endl;                        // affiche l'ouverture de la connexion.
            break;
        case LWS_CALLBACK_CLOSED:                                                               // Traite la fermeture d'une connexion navigateur.
            if (web_client == wsi) web_client = nullptr;                                        // Efface le client courant s'il correspond a la connexion fermee.
            std::cerr << "[WS] Client navigateur deconnecte" << std::endl;                      // affiche la fermeture de la connexion.
            break;
        case LWS_CALLBACK_SERVER_WRITEABLE: {                                                    // Traite le moment ou le serveur peut ecrire vers le navigateur.
            std::string message;                                                                // Reserve une chaine pour le message a transmettre.
            {
                std::lock_guard<std::mutex> lock(ws_message_mutex);                             // Verrouille la file pendant l'extraction du message.
                message.swap(ws_pending_message);                                               // Recupere le message en attente et vide la file en une operation.
            }
            if (!message.empty()) {                                                             // N'envoie rien si aucun message n'est en attente.
                std::vector<unsigned char> buf(LWS_PRE + message.size());                       // Alloue un tampon avec l'espace reserve exige par LWS.
                memcpy(buf.data() + LWS_PRE, message.data(), message.size());                   // Copie le contenu du message dans le tampon d'envoi.
                const int written = lws_write(wsi, buf.data() + LWS_PRE, message.size(), LWS_WRITE_TEXT); // Envoie le message sous forme de texte.
                if (written < 0) {                                                              // Detecte une erreur de transmission WebSocket.
                    std::cerr << "[WS] Echec lws_write" << std::endl;                           // Signale l'echec de l'ecriture.
                    return -1;                                                                  // Indique a libwebsockets que le rappel a echoue.
                }
                if ((size_t)written != message.size()) {                                        // Verifie si le nombre d'octets envoyes est complet.
                    std::cerr << "[WS] Avertissement: message tronque (" << written             // Signale le nombre d'octets effectivement ecrits.
                              << "/" << message.size() << ")" << std::endl;                     // Affiche la taille attendue pour completer le diagnostic.
                } else {                                                                        // Traite le cas ou le message entier a ete envoye.
                    std::cerr << "[WS] Message envoye au navigateur: " << message << std::endl; // affiche le message transmis.
                }
            }
            break;
        }
        case LWS_CALLBACK_RECEIVE: {                                                            // Traite une commande envoyee par le navigateur.
            
            const std::string payload((const char *)in, len);                                   // Copie la charge utile recue dans une chaine de taille connue.
            
            if (payload.find("\"action\":\"add_checkpoint\"") != std::string::npos) {           // Detecte la demande d'ajout d'un point de passage.
                const char *lat_ptr = strstr(payload.c_str(), "\"lat\":");                      // Cherche le champ de latitude dans le JSON recu.
                const char *lng_ptr = strstr(payload.c_str(), "\"lng\":");                      // Cherche le champ de longitude dans le JSON recu.
                
                if (lat_ptr && lng_ptr) {                                                       // Continue seulement si les deux coordonnees existent.
                    const double lat = atof(lat_ptr + 6);                                       // Convertit la valeur suivant le champ latitude.
                    const double lng = atof(lng_ptr + 6);                                       // Convertit la valeur suivant le champ longitude.
                    std::lock_guard<std::mutex> lock(route_mutex);                              // Protege l'ajout a la route partagee.
                    checkpoints.push_back({lat, lng});                                          // Ajoute le point recu a la fin de la route.
                }

            } else if (payload.find("\"action\":\"clear_route\"") != std::string::npos) {       // Detecte la demande d'effacement de route.
                std::lock_guard<std::mutex> lock(route_mutex);                                  // Protege la modification de la route partagee.
                checkpoints.clear();                                                            // Supprime tous les points de passage.

            } else if (payload.find("\"action\":\"manual_move\"") != std::string::npos) {       // Detecte une commande de mouvement manuel.
                const char *dir_ptr = strstr(payload.c_str(), "\"direction\":\"");              // Cherche la valeur du champ direction.
                if (dir_ptr) {                                                                  // Continue uniquement si le champ direction est present.
                    char dir[16] = {};                                                          // Reserve un tampon nul pour recevoir la direction.
                    if (sscanf(dir_ptr + 13, "%15[^\"]", dir) == 1) {                           // Lit la direction sans depasser le tampon.
                        std::lock_guard<std::mutex> lock(route_mutex);                          // Protege la commande manuelle partagee.
                        manual_direction = std::string(dir);                                    // Enregistre la nouvelle direction demandee.
                    }
                }
            }
            break;
        }

        default:
            break; // Ignore les autres evenements libwebsockets.
    }
    return 0; // Indique que le rappel a ete traite sans erreur.
}

static struct lws_protocols protocols[] = {                             // Declare les protocoles servis par le contexte WebSocket.
    { "robot-protocol", callback_websocket, 0, 4096, 0, nullptr, 0 },   // Associe le protocole robot a son rappel et a un tampon de 4096 octets.
    { nullptr, nullptr, 0, 0, 0, nullptr, 0 }                           // Termine le tableau des protocoles par une entree sentinelle.
};

int main() { // Programme principal

    signal(SIGINT, signal_handler); // Associe Ctrl+C au gestionnaire d'arret du robot.

    serial_fd = init_serial(SERIAL_PORT);                           // Ouvre et configure le port serie du recepteur GNSS.
    if (serial_fd == -1) {                                          // Verifie si le port serie a pu etre initialise.
        std::cerr << "Erreur de liaison série UART." << std::endl;  // Signale l'echec d'initialisation du recepteur.
        return EXIT_FAILURE;                                        // Quitte avec un code d'erreur si le GNSS est inaccessible.
    }

    std::thread t_gnss(gnss_io_thread);             // Demarre le thread de lecture et de traitement GNSS.
    std::thread t_motor(motor_control_thread);      // Demarre le thread de commande des moteurs.
    std::thread t_stats(system_stats_thread);       // Demarre le thread de collecte des statistiques.
    std::thread t_web(web_server_thread);           // Demarre le thread qui lance le serveur HTTP.
    
    struct lws_context_creation_info ws_info;       // Declare les options de creation du serveur WebSocket.
    memset(&ws_info, 0, sizeof(ws_info));           // Initialise toutes les options avec des valeurs nulles.
    ws_info.port = WS_PORT;                         // Configure le port TCP d'ecoute du serveur.
    ws_info.protocols = protocols;                  // Associe les protocoles et rappels WebSocket declares plus haut.

    ws_info.iface = nullptr;                        // Ecoute sur toutes les interfaces reseau disponibles.

    struct lws_context *context = lws_create_context(&ws_info);     // Cree le contexte du serveur WebSocket.
    
    if (context == nullptr) {                                       // Verifie si le contexte a ete cree correctement.

        std::cerr << "[WS] Impossible de creer le serveur WebSocket sur le port " << WS_PORT << std::endl; // Signale l'echec de creation du serveur.
        return EXIT_FAILURE;    // Quitte avec un code d'erreur si WebSocket ne peut pas demarrer.
    }

    std::cout << "[SYSTEM] Serveur WebSocket en ecoute sur le port " << WS_PORT << std::endl; // Confirme le demarrage du serveur WebSocket.
    
    while (true) {                                                                  // Maintient le serveur WebSocket actif et traite ses evenements.
        lws_service(context, 5);                                                    // Traite les evenements reseau pendant au plus 5 ms.
        bool pending = false;                                                       // Initialise l'indicateur de message en attente.
        {
            std::lock_guard<std::mutex> lock(ws_message_mutex);                     // Verrouille la file pendant sa lecture.
            pending = !ws_pending_message.empty();                                  // Determine si un message attend d'etre envoye.
        }
        if (pending && web_client != nullptr) lws_callback_on_writable(web_client); // Planifie l'envoi si un message attend et qu'un client est connecte.
    }
    
    lws_context_destroy(context);           // Detruit le contexte WebSocket apres la boucle de service.

    system("pkill -f 'http.server 8080'");  // Arrete le serveur HTTP Python lors de la fermeture.

    if (t_web.joinable()) {                 // Verifie que le thread web peut etre rejoint.
        t_web.join();                       // Attend la fin du thread web.
    }

    t_gnss.join();                          // Attend la fin du thread GNSS.
    t_motor.join();                         // Attend la fin du thread de commande des moteurs.
    t_stats.join();                         // Attend la fin du thread de statistiques.
    
    return 0;                               // Termine le programme.
}