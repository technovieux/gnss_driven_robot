#include "robot_state.h"            // Donne acces a la route, aux coordonnees et aux commandes partagees.
#include "thread_functions.h"       // Rend visible la declaration du thread de commande des moteurs.

#include "lib/adafruitmotorhat.h"   // Fournit l'interface materielle du Motor HAT Adafruit.

#include <cmath>                    // Fournit les fonctions mathematiques pour distance et angle.
#include <iostream>                 // Permet de signaler les erreurs de materiel.
#include <unistd.h>                 // Fournit usleep pour temporiser les cycles de commande.

namespace {                         // Limite les constantes materieles a cette unite de compilation.

constexpr int MOTOR_LEFT_F_PORT = 1;    // Numero du moteur avant gauche sur la carte.
constexpr int MOTOR_LEFT_B_PORT = 2;    // Numero du moteur arriere gauche sur la carte.
constexpr int MOTOR_RIGHT_F_PORT = 3;   // Numero du moteur avant droit sur la carte.
constexpr int MOTOR_RIGHT_B_PORT = 4;   // Numero du moteur arriere droit sur la carte.
constexpr int MOTOR_SPEED = 100;        // Vitesse utilisee lors des deplacements manuels.

}

void motor_control_thread() {                               // Pilote les moteurs selon les commandes manuelles ou la route GNSS.
    AdafruitMotorHAT hat;                                   // Initialise l'interface avec la carte de commande des moteurs.
    auto motor_left_F = hat.getMotor(MOTOR_LEFT_F_PORT);    // Recupere le moteur avant gauche.
    auto motor_left_B = hat.getMotor(MOTOR_LEFT_B_PORT);    // Recupere le moteur arriere gauche.
    auto motor_right_F = hat.getMotor(MOTOR_RIGHT_F_PORT);  // Recupere le moteur avant droit.
    auto motor_right_B = hat.getMotor(MOTOR_RIGHT_B_PORT);  // Recupere le moteur arriere droit.
    
    if (!motor_left_F || !motor_left_B || !motor_right_F || !motor_right_B) {           // Verifie que les quatre moteurs sont disponibles.
        std::cerr << "[MOTEURS] Erreur matérielle Adafruit Motor HAT !" << std::endl;   // Signale qu'un moteur n'a pas pu etre initialise.
        return;     // Arrete le thread si la configuration materielle est incomplete.
    }

    while (true) {                                              // Recalcule les commandes a envoyer aux moteurs.
        route_mutex.lock();                                     // Verrouille les donnees de route partagees avec les autres threads.
        std::string mode_manuel = manual_direction;             // Copie la commande manuelle courante sous protection du verrou.

        if (mode_manuel != "STOP") {                            // Donne la priorite a une commande manuelle active.
            route_mutex.unlock();                               // Libere le verrou avant de commander les moteurs.
            motor_left_F->setSpeed(MOTOR_SPEED);                // Applique la vitesse manuelle au moteur avant gauche.
            motor_left_B->setSpeed(MOTOR_SPEED);                // Applique la vitesse manuelle au moteur arriere gauche.
            motor_right_F->setSpeed(MOTOR_SPEED);               // Applique la vitesse manuelle au moteur avant droit.
            motor_right_B->setSpeed(MOTOR_SPEED);               // Applique la vitesse manuelle au moteur arriere droit.

            if (mode_manuel == "AV") {                          // Applique la combinaison de sens correspondant a l'avance.
                motor_left_F->run(AdafruitDCMotor::kBackward);  // Fait tourner le moteur gauche avant dans son sens d'avance.
                motor_left_B->run(AdafruitDCMotor::kBackward);  // Fait tourner le moteur gauche arriere dans son sens d'avance.
                motor_right_F->run(AdafruitDCMotor::kForward);  // Fait tourner le moteur droit avant dans son sens d'avance.
                motor_right_B->run(AdafruitDCMotor::kForward);  // Fait tourner le moteur droit arriere dans son sens d'avance.

            } else if (mode_manuel == "AR") {                   // Applique la combinaison de sens correspondant au recul.
                motor_left_F->run(AdafruitDCMotor::kForward);   // Fait tourner le moteur gauche avant en marche arriere.
                motor_left_B->run(AdafruitDCMotor::kForward);   // Fait tourner le moteur gauche arriere en marche arriere.
                motor_right_F->run(AdafruitDCMotor::kBackward); // Fait tourner le moteur droit avant en marche arriere.
                motor_right_B->run(AdafruitDCMotor::kBackward); // Fait tourner le moteur droit arriere en marche arriere.

            } else if (mode_manuel == "G") {                    // Applique le sens de rotation pour tourner a gauche.
                motor_left_F->run(AdafruitDCMotor::kForward);   // Fait tourner le moteur gauche avant vers l'arriere.
                motor_left_B->run(AdafruitDCMotor::kForward);   // Fait tourner le moteur gauche arriere vers l'arriere.
                motor_right_F->run(AdafruitDCMotor::kForward);  // Fait tourner le moteur droit avant vers l'avant.
                motor_right_B->run(AdafruitDCMotor::kForward);  // Fait tourner le moteur droit arriere vers l'avant.

            } else if (mode_manuel == "D") {                    // Applique le sens de rotation pour tourner a droite.
                motor_left_F->run(AdafruitDCMotor::kBackward);  // Fait tourner le moteur gauche avant vers l'avant.
                motor_left_B->run(AdafruitDCMotor::kBackward);  // Fait tourner le moteur gauche arriere vers l'avant.
                motor_right_F->run(AdafruitDCMotor::kBackward); // Fait tourner le moteur droit avant vers l'arriere.
                motor_right_B->run(AdafruitDCMotor::kBackward); // Fait tourner le moteur droit arriere vers l'arriere.

            }
            usleep(50000);
            continue; // Recommence le cycle sans appliquer le guidage automatique.
        }

        if (!checkpoints.empty() && current_position.lat != 0.0) {  // Demarre le guidage si une destination et une position existent.
            Point target = checkpoints.front();                     // Copie le prochain point a atteindre.
            Point current = current_position;                       // Copie la position actuelle du robot.
            double heading = current_heading;                       // Copie le cap courant estime par le GNSS.
            int gnss_mode = current_gnss_mode;                      // Copie le niveau de qualite de la position GNSS.
            route_mutex.unlock();                                   // Libere les donnees partagees avant les calculs et commandes.

            if (gnss_mode < 4) {                                    // Arret du robot si la qualite GNSS n'est pas RTK Fix ou Float.
                motor_left_F->run(AdafruitDCMotor::kRelease);       // Libere le moteur gauche avant.
                motor_left_B->run(AdafruitDCMotor::kRelease);       // Libere le moteur gauche arriere.
                motor_right_F->run(AdafruitDCMotor::kRelease);      // Libere le moteur droit avant.
                motor_right_B->run(AdafruitDCMotor::kRelease);      // Libere le moteur droit arriere.
                
                usleep(50000);
                continue; // Recommence sans calculer de direction.
            }

            double distance = sqrt(pow(target.lat - current.lat, 2) + pow(target.lng - current.lng, 2));        // Calcule une distance angulaire intermediaire.
            double distance_deg = sqrt(pow(target.lat - current.lat, 2) + pow(target.lng - current.lng, 2));    // Calcule l'ecart en degres entre la position et la cible.
            double distance_meters = distance_deg * 111320.0;                                                   // Convertit approximativement l'ecart en metres.

            if (distance_meters <= 1.5) {                               // Considere la destination atteinte dans un rayon de 1,5 m.
                route_mutex.lock();                                     // Verrouille la route avant d'en retirer le point atteint.
                checkpoints.erase(checkpoints.begin());                 // Retire le point de passage en tete de route.
                route_mutex.unlock();                                   // Libere la route apres sa mise a jour.
                motor_left_F->run(AdafruitDCMotor::kRelease);           // Arrete le moteur gauche avant.
                motor_right_F->run(AdafruitDCMotor::kRelease);          // Arrete le moteur droit avant.
                motor_left_B->run(AdafruitDCMotor::kRelease);           // Arrete le moteur gauche arriere.
                motor_right_B->run(AdafruitDCMotor::kRelease);          // Arrete le moteur droit arriere.
                usleep(500000);                                         // Attend une demi-seconde avant de poursuivre la route.
                continue;                                               // Recommence avec le prochain point de passage.
            }

            double target_angle = atan2(target.lng - current.lng, target.lat - current.lat) * 180.0 / M_PI;     // Calcule le cap necessaire vers le point cible.
            double angle_error = target_angle - heading;                                                        // Calcule l'ecart entre le cap souhaite et le cap courant.
            if (angle_error > 180.0) angle_error -= 360.0;                                                      // Ramene un ecart positif dans l'intervalle des caps courts.
            if (angle_error < -180.0) angle_error += 360.0;                                                     // Ramene un ecart negatif dans l'intervalle des caps courts.

            int base_speed = MOTOR_SPEED;                       // Definit la vitesse nominale du robot en guidage automatique.
            double kp = 3.5;                                    // Definit le gain de correction proportionnelle du cap.
            int speed_correction = (int)(angle_error * kp);     // Convertit l'erreur de cap en correction de vitesse.
            int speed_l = base_speed + speed_correction;        // Calcule la vitesse corrigee du cote gauche.
            int speed_r = base_speed - speed_correction;        // Calcule la vitesse corrigee du cote droit.

            if (speed_l > 255) speed_l = 255;   // Limite la vitesse gauche a la valeur maximale du controleur.
            if (speed_l < 0) speed_l = 0;       // Empeche une vitesse gauche negative.
            if (speed_r > 255) speed_r = 255;   // Limite la vitesse droite a la valeur maximale du controleur.
            if (speed_r < 0) speed_r = 0;       // Empeche une vitesse droite negative.

            motor_left_F->setSpeed(speed_l);                // Applique la vitesse corrigee au moteur gauche avant.
            motor_left_B->setSpeed(speed_l);                // Applique la vitesse corrigee au moteur gauche arriere.
            motor_right_F->setSpeed(speed_r);               // Applique la vitesse corrigee au moteur droit avant.
            motor_right_B->setSpeed(speed_r);               // Applique la vitesse corrigee au moteur droit arriere.
            motor_left_F->run(AdafruitDCMotor::kForward);   // Commande l'avance du moteur gauche avant.
            motor_left_B->run(AdafruitDCMotor::kForward);   // Commande l'avance du moteur gauche arriere.
            motor_right_F->run(AdafruitDCMotor::kForward);  // Commande l'avance du moteur droit avant.
            motor_right_B->run(AdafruitDCMotor::kForward);  // Commande l'avance du moteur droit arriere.
        } 
        else {                                              // Traite le cas ou aucune route exploitable n'est disponible.
            route_mutex.unlock();                           // Libere le verrou pris au debut du cycle.
            motor_left_F->run(AdafruitDCMotor::kRelease);   // Libere le moteur gauche avant en l'absence de cible.
            motor_left_B->run(AdafruitDCMotor::kRelease);   // Libere le moteur gauche arriere en l'absence de cible.
            motor_right_F->run(AdafruitDCMotor::kRelease);  // Libere le moteur droit avant en l'absence de cible.
            motor_right_B->run(AdafruitDCMotor::kRelease);  // Libere le moteur droit arriere en l'absence de cible.
        }
        usleep(50000);                                      // Attend 50 ms avant le prochain cycle de controle.
    }
}
