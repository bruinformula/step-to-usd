#pragma once

#include <iostream>
#include <mutex>
#include <chrono>
#include <string>

class Logger {
public:
    enum Level { INFO, DEBUG, ERR, NONE, WARN };
    inline static Level activeLevel = INFO;
    
    static void log(Level lvl, const std::string& msg) {
        if (activeLevel == NONE) return;
        if (lvl == DEBUG && activeLevel != DEBUG) return;
        
        static std::mutex mtx; // Thread safe outputs
        std::lock_guard<std::mutex> lock(mtx);
        
        if (lvl == ERR) {
        
        } else if (lvl == WARN) {
            std::cerr << "[WARNING] " << msg << std::endl;
        } else if (lvl == ERR) {
            std::cerr << "[ERROR] " << msg << std::endl;
        } else if (lvl == DEBUG) {
            std::cout << "[DEBUG] " << msg << std::endl;
        } else {
            if (activeLevel == DEBUG) {
                std::cout << "[INFO] " << msg << std::endl;
            } else {
                std::cout << msg << std::endl;
            }
        }
    }

    static void progress(int current, int total, const std::string& label) {
        if (activeLevel == NONE || activeLevel == DEBUG) return;
        static std::mutex mtx; // Thread safe outputs
        std::lock_guard<std::mutex> lock(mtx);
        std::cerr << "\r[" << current << "/" << total << "] " << label << "..." << std::flush;
    }

    static void progressDone() {
        if (activeLevel == NONE || activeLevel == DEBUG) return;
        static std::mutex mtx; // Thread safe outputs
        std::lock_guard<std::mutex> lock(mtx);
        std::cerr << std::endl;
    }
};

class ScopedTimer {
    std::string name;
    std::chrono::high_resolution_clock::time_point start;
public:
    ScopedTimer(const std::string& n) : name(n), start(std::chrono::high_resolution_clock::now()) {
        Logger::log(Logger::DEBUG, "Start: " + name); 
    }
    ~ScopedTimer() {
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end - start;
        Logger::log(Logger::DEBUG, "Done: " + name + " in " + std::to_string(diff.count()) + "s");
    }
};

#define LOG_INFO(msg) Logger::log(Logger::INFO, msg)
#define LOG_DEBUG(msg) Logger::log(Logger::DEBUG, msg)
#define LOG_ERR(msg) Logger::log(Logger::ERR, msg)
#define LOG_WARN(msg) Logger::log(Logger::WARN, msg)

#define LOG_PROGRESS(current, total, label) Logger::progress(current, total, label)
#define LOG_PROGRESS_DONE() Logger::progressDone()

#define LOG_SCOPED_TIMER(name) ScopedTimer timer_##__LINE__(name)
