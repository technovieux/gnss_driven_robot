#include "thread_functions.h"

#include <cstdlib>
#include <iostream>

void web_server_thread() {
    int ret = system("python3 -m http.server 8080 &");
    if (ret != 0) {
        std::cerr << "[WEB] Erreur lors du lancement du serveur web Python" << std::endl;
    } else {
        std::cout << "[WEB] Serveur web sur http://localhost:8080" << std::endl;
    }
}
