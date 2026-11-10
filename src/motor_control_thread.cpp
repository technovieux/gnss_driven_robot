#include "robot_state.h"
#include "thread_functions.h"

#include "lib/adafruitmotorhat.h"

#include <cmath>
#include <iostream>
#include <unistd.h>

namespace {

constexpr int MOTOR_LEFT_F_PORT = 1;
constexpr int MOTOR_LEFT_B_PORT = 2;
constexpr int MOTOR_RIGHT_F_PORT = 3;
constexpr int MOTOR_RIGHT_B_PORT = 4;
constexpr int MOTOR_SPEED = 100;

}

void motor_control_thread() {
    AdafruitMotorHAT hat;
    auto motor_left_F = hat.getMotor(MOTOR_LEFT_F_PORT);
    auto motor_left_B = hat.getMotor(MOTOR_LEFT_B_PORT);
    auto motor_right_F = hat.getMotor(MOTOR_RIGHT_F_PORT);
    auto motor_right_B = hat.getMotor(MOTOR_RIGHT_B_PORT);
    if (!motor_left_F || !motor_left_B || !motor_right_F || !motor_right_B) {
        std::cerr << "[MOTEURS] Erreur matérielle Adafruit Motor HAT !" << std::endl;
        return;
    }

    while (true) {
        route_mutex.lock();
        std::string mode_manuel = manual_direction;

        if (mode_manuel != "STOP") {
            route_mutex.unlock();
            motor_left_F->setSpeed(MOTOR_SPEED);
            motor_left_B->setSpeed(MOTOR_SPEED);
            motor_right_F->setSpeed(MOTOR_SPEED);
            motor_right_B->setSpeed(MOTOR_SPEED);

            if (mode_manuel == "AV") {
                motor_left_F->run(AdafruitDCMotor::kBackward);
                motor_left_B->run(AdafruitDCMotor::kBackward);
                motor_right_F->run(AdafruitDCMotor::kForward);
                motor_right_B->run(AdafruitDCMotor::kForward);
            } else if (mode_manuel == "AR") {
                motor_left_F->run(AdafruitDCMotor::kForward);
                motor_left_B->run(AdafruitDCMotor::kForward);
                motor_right_F->run(AdafruitDCMotor::kBackward);
                motor_right_B->run(AdafruitDCMotor::kBackward);
            } else if (mode_manuel == "G") {
                motor_left_F->run(AdafruitDCMotor::kForward);
                motor_left_B->run(AdafruitDCMotor::kForward);
                motor_right_F->run(AdafruitDCMotor::kForward);
                motor_right_B->run(AdafruitDCMotor::kForward);
            } else if (mode_manuel == "D") {
                motor_left_F->run(AdafruitDCMotor::kBackward);
                motor_left_B->run(AdafruitDCMotor::kBackward);
                motor_right_F->run(AdafruitDCMotor::kBackward);
                motor_right_B->run(AdafruitDCMotor::kBackward);
            }
            usleep(50000);
            continue;
        }

        if (!checkpoints.empty() && current_position.lat != 0.0) {
            Point target = checkpoints.front();
            Point current = current_position;
            double heading = current_heading;
            int gnss_mode = current_gnss_mode;
            route_mutex.unlock();

            if (gnss_mode < 4) {
                motor_left_F->run(AdafruitDCMotor::kRelease);
                motor_left_B->run(AdafruitDCMotor::kRelease);
                motor_right_F->run(AdafruitDCMotor::kRelease);
                motor_right_B->run(AdafruitDCMotor::kRelease);
                usleep(50000);
                continue;
            }

            double distance = sqrt(pow(target.lat - current.lat, 2) + pow(target.lng - current.lng, 2));
            double distance_deg = sqrt(pow(target.lat - current.lat, 2) + pow(target.lng - current.lng, 2));
            double distance_meters = distance_deg * 111320.0;

            if (distance_meters <= 1.5) {
                route_mutex.lock();
                checkpoints.erase(checkpoints.begin());
                route_mutex.unlock();
                motor_left_F->run(AdafruitDCMotor::kRelease);
                motor_right_F->run(AdafruitDCMotor::kRelease);
                motor_left_B->run(AdafruitDCMotor::kRelease);
                motor_right_B->run(AdafruitDCMotor::kRelease);
                usleep(500000);
                continue;
            }

            double target_angle = atan2(target.lng - current.lng, target.lat - current.lat) * 180.0 / M_PI;
            double angle_error = target_angle - heading;
            if (angle_error > 180.0) angle_error -= 360.0;
            if (angle_error < -180.0) angle_error += 360.0;

            int base_speed = MOTOR_SPEED;
            double kp = 3.5;
            int speed_correction = (int)(angle_error * kp);
            int speed_l = base_speed + speed_correction;
            int speed_r = base_speed - speed_correction;

            if (speed_l > 255) speed_l = 255; if (speed_l < 0) speed_l = 0;
            if (speed_r > 255) speed_r = 255; if (speed_r < 0) speed_r = 0;

            motor_left_F->setSpeed(speed_l);
            motor_left_B->setSpeed(speed_l);
            motor_right_F->setSpeed(speed_r);
            motor_right_B->setSpeed(speed_r);
            motor_left_F->run(AdafruitDCMotor::kForward);
            motor_left_B->run(AdafruitDCMotor::kForward);
            motor_right_F->run(AdafruitDCMotor::kForward);
            motor_right_B->run(AdafruitDCMotor::kForward);
        } else {
            route_mutex.unlock();
            motor_left_F->run(AdafruitDCMotor::kRelease);
            motor_left_B->run(AdafruitDCMotor::kRelease);
            motor_right_F->run(AdafruitDCMotor::kRelease);
            motor_right_B->run(AdafruitDCMotor::kRelease);
        }
        usleep(50000);
    }
}
