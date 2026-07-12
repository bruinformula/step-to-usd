#pragma once

#include <iostream>
#include <mutex>
#include <chrono>
#include <string>

class Logger {
public:
    enum Level { INFO, DEBUG, ERR, NONE, WARN };
    inline static Level activeLevel = INFO;

private:
    // Track current progress state so we can redraw after interruptions
    inline static std::string progressLabel;
    inline static int progressCurrent = 0;
    inline static int progressTotal = 0;
    inline static bool progressActive = false;
    inline static std::mutex mtx; // Single shared mutex for output

    static void clearProgress() {
        if (progressActive)
            std::cerr << "\r\033[K" << std::flush; // \033[K = erase to end of line
    }

    static void redrawProgress() {
        if (progressActive)
            std::cerr << "\r[" << progressCurrent << "/" << progressTotal << "] " 
                      << progressLabel << "..." << std::flush;
    }

public:
    static void log(Level lvl, const std::string& msg) {
        if (activeLevel == NONE) return;
        if (lvl == DEBUG && activeLevel != DEBUG) return;

        std::lock_guard<std::mutex> lock(mtx);
        clearProgress();

        if (lvl == WARN) {
            std::cerr << "[WARNING] " << msg << std::endl;
        } else if (lvl == ERR) {
            std::cerr << "[ERROR] " << msg << std::endl;
        } else if (lvl == DEBUG) {
            std::cout << "[DEBUG] " << msg << std::endl;
        } else {
            if (activeLevel == DEBUG)
                std::cout << "[INFO] " << msg << std::endl;
            else
                std::cout << msg << std::endl;
        }

        redrawProgress();
    }

    static void progress(int current, int total, const std::string& label) {
        if (activeLevel == NONE || activeLevel == DEBUG) return;

        std::lock_guard<std::mutex> lock(mtx);
        progressCurrent = current;
        progressTotal = total;
        progressLabel = label;
        progressActive = true;
        redrawProgress();
    }

    static void progressDone() {
        if (activeLevel == NONE || activeLevel == DEBUG) return;

        std::lock_guard<std::mutex> lock(mtx);
        progressActive = false;
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
