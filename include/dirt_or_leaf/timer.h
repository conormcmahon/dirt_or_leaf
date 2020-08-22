
#ifndef TIMER_
#define TIMER_

#include <iostream>
#include <chrono>

class Timer
{
private:
    std::chrono::system_clock::time_point start_time_;
    std::chrono::system_clock::time_point stop_time_;
    std::string process_name_;

public:
    Timer(std::string process_name)
    {
        process_name_ = process_name;
        start_time_ = std::chrono::high_resolution_clock::now();
    }
    float stop()
    {
        stop_time_ = std::chrono::high_resolution_clock::now();
        float duration = std::chrono::duration_cast<std::chrono::microseconds>( stop_time_ - start_time_ ).count();
        std::cout << "Execution time for process " << process_name_ << " was " << duration/1000000 << std::endl; 
        return duration;
    }
};

#endif // TIMER_