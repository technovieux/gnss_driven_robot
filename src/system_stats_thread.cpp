#include "robot_state.h"
#include "thread_functions.h"

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>

namespace {

void queue_ws_message(const std::string &message) {
    std::lock_guard<std::mutex> lock(ws_message_mutex);
    ws_pending_message = message;
}

bool read_cpu_counters(uint64_t &total, uint64_t &idle) {
    std::ifstream stat_file("/proc/stat");
    if (!stat_file) {
        std::cerr << "[STATS] Impossible de lire /proc/stat" << std::endl;
        return false;
    }

    std::string label;
    if (!(stat_file >> label) || label != "cpu") {
        std::cerr << "[STATS] Compteurs CPU absents de /proc/stat" << std::endl;
        return false;
    }

    std::vector<uint64_t> counters;
    uint64_t value;
    while (stat_file >> value) counters.push_back(value);
    if (counters.size() < 4) {
        std::cerr << "[STATS] Compteurs CPU incomplets dans /proc/stat" << std::endl;
        return false;
    }

    total = 0;
    for (size_t i = 0; i < counters.size() && i < 8; ++i) total += counters[i];
    idle = counters[3] + (counters.size() > 4 ? counters[4] : 0);
    return true;
}

bool read_memory_counters(uint64_t &total_kb, uint64_t &available_kb) {
    std::ifstream meminfo_file("/proc/meminfo");
    if (!meminfo_file) {
        std::cerr << "[STATS] Impossible de lire /proc/meminfo" << std::endl;
        return false;
    }

    std::string line;
    std::string label;
    uint64_t value;
    bool has_total = false;
    bool has_available = false;
    while (std::getline(meminfo_file, line)) {
        std::istringstream entry(line);
        if (!(entry >> label >> value)) continue;
        if (label == "MemTotal:") {
            total_kb = value;
            has_total = true;
        } else if (label == "MemAvailable:") {
            available_kb = value;
            has_available = true;
        }
    }

    if (!has_total || !has_available || total_kb == 0 || available_kb > total_kb) {
        std::cerr << "[STATS] Informations mémoire incomplètes ou invalides dans /proc/meminfo" << std::endl;
        return false;
    }
    return true;
}

}

void system_stats_thread() {
    uint64_t previous_total = 0;
    uint64_t previous_idle = 0;
    bool has_previous_sample = false;

    while (true) {
        sleep(1);

        uint64_t total = 0;
        uint64_t idle = 0;
        uint64_t memory_total_kb = 0;
        uint64_t memory_available_kb = 0;
        if (!read_cpu_counters(total, idle) ||
            !read_memory_counters(memory_total_kb, memory_available_kb)) {
            continue;
        }

        if (has_previous_sample && total > previous_total && idle >= previous_idle) {
            const uint64_t total_delta = total - previous_total;
            const uint64_t idle_delta = idle - previous_idle;
            const double cpu_percent = 100.0 * (total_delta - idle_delta) / total_delta;
            const double ram_percent = 100.0 * (memory_total_kb - memory_available_kb) / memory_total_kb;

            std::ostringstream message;
            message << std::fixed << std::setprecision(1)
                    << "{\"type\":\"system_stats\",\"cpu_percent\":" << cpu_percent
                    << ",\"ram_percent\":" << ram_percent << "}";
            queue_ws_message(message.str());
        } else if (has_previous_sample) {
            std::cerr << "[STATS] Compteurs CPU incohérents, nouvel échantillon de référence" << std::endl;
        }

        previous_total = total;
        previous_idle = idle;
        has_previous_sample = true;
    }
}
