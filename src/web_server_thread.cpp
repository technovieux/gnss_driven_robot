#include "thread_functions.h" // Rend visible la declaration de la fonction du thread web.

#include <cstdlib>          // Declare system pour lancer le serveur HTTP.
#include <iostream>         // Permet d'afficher les messages de demarrage et d'erreur.

void web_server_thread() {                                                                  // Demarre le serveur HTTP utilise par l'interface web.
    int ret = system("python3 -m http.server 8080 &");                                      // Lance le serveur Python en arriere-plan sur le port 8080.
    if (ret != 0) {
        std::cerr << "[WEB] Erreur lors du lancement du serveur web Python" << std::endl;   // Signale l'echec dans la sortie d'erreur.
    } else {
        std::cout << "[WEB] Serveur web sur http://localhost:8080" << std::endl;            // Indique l'adresse locale du serveur.
    }
}
