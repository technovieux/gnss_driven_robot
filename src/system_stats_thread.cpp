#include "robot_state.h"        // Donne acces au verrou et a la file de messages partages.
#include "thread_functions.h"   // Rend visible la declaration du thread de statistiques.

#include <cstdint>  // Fournit les entiers de taille fixe pour les compteurs systeme.
#include <fstream>  // Permet de lire les fichiers de statistiques Linux.
#include <iomanip>  // Fournit le formatage decimal des mesures.
#include <iostream> // Permet de signaler les erreurs et incoherences.
#include <sstream>  // Permet de construire le message JSON des statistiques.
#include <string>   // Fournit le type texte des etiquettes et messages.
#include <vector>   // Permet de conserver les compteurs CPU lus.
#include <unistd.h> // Fournit sleep pour temporiser la boucle du thread.

namespace { // Limite les fonctions utilitaires a ce fichier.

void queue_ws_message(const std::string &message) {         // Place le dernier message dans la file partagee.
    std::lock_guard<std::mutex> lock(ws_message_mutex);     // Protege l'acces concurrent a la file.
    ws_pending_message = message;                           // Remplace le message en attente par le plus recent.
}

bool read_cpu_counters(uint64_t &total, uint64_t &idle) {   // Lit et totalise les compteurs CPU du noyau.
    std::ifstream stat_file("/proc/stat");                  // Ouvre le fichier Linux contenant les compteurs CPU.
    if (!stat_file) {                                       // Verifie que le fichier a pu etre ouvert.
        std::cerr << "[STATS] Impossible de lire /proc/stat" << std::endl;  // Signale l'impossibilite de lire les compteurs.
        return false;                                       // Indique a l'appelant que l'echantillon est inutilisable.
    }

    std::string label;                                                              // Recoit le nom de la premiere ligne du fichier.
    if (!(stat_file >> label) || label != "cpu") {                                  // Verifie que la premiere ligne contient les compteurs CPU globaux.
        std::cerr << "[STATS] Compteurs CPU absents de /proc/stat" << std::endl;    // Signale une ligne CPU absente ou invalide.
        return false;                                                               // Refuse d'utiliser un echantillon mal forme.
    }

    std::vector<uint64_t> counters;                                                     // Conserve les compteurs individuels lus sur la ligne CPU.
    uint64_t value;                                                                     // Stocke temporairement chaque valeur lue.
    while (stat_file >> value) counters.push_back(value);                               // Lit toutes les valeurs numeriques disponibles.
    if (counters.size() < 4) {                                                          // Verifie la presence des compteurs minimaux necessaires au calcul.
        std::cerr << "[STATS] Compteurs CPU incomplets dans /proc/stat" << std::endl;   // Signale la ligne CPU incomplete.
        return false;                                                                   // Indique qu'aucun echantillon valide n'a ete produit.
    }

    total = 0;                                                                          // Reinitialise le total avant d'additionner les compteurs.
    for (size_t i = 0; i < counters.size() && i < 8; ++i) total += counters[i];         // Additionne les huit champs CPU connus au maximum.
    idle = counters[3] + (counters.size() > 4 ? counters[4] : 0);                       // Additionne le temps inactif et le temps d'attente I/O.
    return true;                                                                        // Indique que les deux compteurs ont ete lus correctement.
}

bool read_memory_counters(uint64_t &total_kb, uint64_t &available_kb) {         // Lit la memoire totale et disponible en kilo-octets.
    std::ifstream meminfo_file("/proc/meminfo");                                // Ouvre le fichier Linux contenant les informations memoire.
    if (!meminfo_file) {                                                        // Verifie que le fichier memoire est accessible.
        std::cerr << "[STATS] Impossible de lire /proc/meminfo" << std::endl;   // Signale l'echec de lecture du fichier.
        return false;                                                           // Indique a l'appelant que les valeurs sont indisponibles.
    }

    std::string line;                               // Recoit chaque ligne lue dans /proc/meminfo.
    std::string label;                              // Recoit le nom du champ present sur la ligne courante.
    uint64_t value;                                 // Recoit la valeur numerique du champ courant.
    bool has_total = false;                         // Indique si MemTotal a ete trouve.
    bool has_available = false;                     // Indique si MemAvailable a ete trouve.
    while (std::getline(meminfo_file, line)) {      // Parcourt toutes les lignes du fichier memoire.
        std::istringstream entry(line);             // Prepare la ligne pour en extraire ses champs.
        if (!(entry >> label >> value)) continue;   // Ignore les lignes sans etiquette et valeur numerique.
        if (label == "MemTotal:") {                 // Detecte la quantite totale de memoire.
            total_kb = value;                       // Enregistre la memoire totale en kilo-octets.
            has_total = true;                       // Memorise la presence du champ obligatoire.
        } else if (label == "MemAvailable:") {      // Detecte la memoire utilisable estimee par le noyau.
            available_kb = value;                   // Enregistre la memoire disponible en kilo-octets.
            has_available = true;                   // Memorise la presence du champ obligatoire.
        }
    }

    if (!has_total || !has_available || total_kb == 0 || available_kb > total_kb) {                             // Rejette les valeurs absentes, nulles ou incoherentes.
        std::cerr << "[STATS] Informations mémoire incomplètes ou invalides dans /proc/meminfo" << std::endl;   // Signale les donnees memoire invalides.
        return false;                                                                                           // Indique que le calcul du pourcentage est impossible.
    }
    return true;                                                                                                // Confirme que les deux compteurs memoire sont valides.
}

}

void system_stats_thread() {                // Echantillonne le CPU et la memoire pour informer l'interface.
    uint64_t previous_total = 0;            // Conserve le total CPU de l'echantillon precedent.
    uint64_t previous_idle = 0;             // Conserve le temps CPU inactif precedent.
    bool has_previous_sample = false;       // Distingue le premier echantillon des suivants.

    while (true) {                          // Repete la collecte tant que le programme fonctionne.
        sleep(1);                           // Attend une seconde entre deux mesures.

        uint64_t total = 0;                                                 // Recoit le total courant des temps CPU.
        uint64_t idle = 0;                                                  // Recoit le temps courant passe au repos.
        uint64_t memory_total_kb = 0;                                       // Recoit la capacite memoire totale.
        uint64_t memory_available_kb = 0;                                   // Recoit la capacite memoire disponible.
        if (!read_cpu_counters(total, idle) ||                              // Lit les compteurs CPU ou detecte une erreur de lecture.
            !read_memory_counters(memory_total_kb, memory_available_kb)) {  // Lit les compteurs memoire ou detecte une erreur.
            continue;                                                       // Ignore cet echantillon incomplet et recommence la boucle.
        }

        if (has_previous_sample && total > previous_total && idle >= previous_idle) {                       // Calcule l'utilisation quand deux echantillons coherents existent.
            const uint64_t total_delta = total - previous_total;                                            // Calcule la progression totale des compteurs CPU.
            const uint64_t idle_delta = idle - previous_idle;                                               // Calcule la progression du temps CPU inactif.
            const double cpu_percent = 100.0 * (total_delta - idle_delta) / total_delta;                    // Convertit le temps CPU actif en pourcentage.
            const double ram_percent = 100.0 * (memory_total_kb - memory_available_kb) / memory_total_kb;   // Calcule la fraction de memoire utilisee.

            std::ostringstream message;                                                                     // Cree le flux servant a formater le message JSON.
            message << std::fixed << std::setprecision(1)                                                   // Formate les pourcentages avec une decimale.
                    << "{\"type\":\"system_stats\",\"cpu_percent\":" << cpu_percent                         // Ajoute le type du message et le pourcentage CPU.
                    << ",\"ram_percent\":" << ram_percent << "}";                                           // Ajoute le pourcentage memoire et ferme l'objet JSON.
            queue_ws_message(message.str());                                                                // Met les statistiques formatees en attente pour le WebSocket.
        } 
        else if (has_previous_sample) {                                                                     // Signale une incoherence apres le premier echantillon.
            std::cerr << "[STATS] Compteurs CPU incohérents, nouvel échantillon de référence" << std::endl; // Avertit que les compteurs CPU ont regresse.
        }

        previous_total = total;             // Enregistre le total CPU pour le prochain calcul.
        previous_idle = idle;               // Enregistre le temps inactif pour le prochain calcul.
        has_previous_sample = true;         // Indique qu'un echantillon de reference est disponible.
    }
}
