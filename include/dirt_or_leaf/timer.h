

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
    void stop()
    {
        stop_time_ = std::chrono::high_resolution_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::microseconds>( stop_time_ - start_time_ ).count();
        std::cout << "\n  Execution time for process " << process_name_ << " was " << duration/1000000; 
    }
};