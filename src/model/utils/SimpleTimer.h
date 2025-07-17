#pragma once
#include <chrono>
#include <iostream>

class SimpleTimer {
public:
    SimpleTimer() { reset(); }
    void reset() { m_start = std::chrono::high_resolution_clock::now(); }
    double elapsedSeconds() const {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = now - m_start;
        return diff.count();
    }
private:
    std::chrono::high_resolution_clock::time_point m_start;
};
