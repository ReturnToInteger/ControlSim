#pragma once

#include <chrono>
#include <iostream>

class Timer {
public:
    Timer() { reset(); }
    void reset() { _start = std::chrono::high_resolution_clock::now(); }
    double elapsedSeconds() const {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = now - _start;
        return diff.count();
    }
private:
    std::chrono::high_resolution_clock::time_point _start;
};
