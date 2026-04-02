#pragma once

#include <iostream>
#include <mutex>
#include <chrono>
#include <string>

class Logger {
public:
    enum Level { INFO, VERBOSE, ERR };
    static Level activeLevel;
    
    static void log(Level lvl, const std::string& msg) {
        if (lvl == VERBOSE && activeLevel != VERBOSE) return;
        
        static std::mutex mtx; // Thread safe outputs
        std::lock_guard<std::mutex> lock(mtx);
        
        if (lvl == ERR) {
            std::cerr << "[ERROR] " << msg << std::endl;
        } else if (lvl == VERBOSE) {
            std::cout << "[VERBOSE] " << msg << std::endl;
        } else {
            std::cout << "[INFO] " << msg << std::endl;
        }
    }
};

class ScopedTimer {
    std::string name;
    std::chrono::high_resolution_clock::time_point start;
public:
    ScopedTimer(const std::string& n) : name(n), start(std::chrono::high_resolution_clock::now()) {
        Logger::log(Logger::VERBOSE, "Start: " + name); 
    }
    ~ScopedTimer() {
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end - start;
        Logger::log(Logger::INFO, "Done: " + name + " in " + std::to_string(diff.count()) + "s");
    }
};

#define LOG_INFO(msg) Logger::log(Logger::INFO, msg)
#define LOG_VERB(msg) Logger::log(Logger::VERBOSE, msg)
#define LOG_ERR(msg) Logger::log(Logger::ERR, msg)
#define LOG_SCOPED_TIMER(name) ScopedTimer timer_##__LINE__(name)
