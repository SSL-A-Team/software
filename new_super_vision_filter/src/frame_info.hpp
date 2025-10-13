#include <chrono>

class LastFrameInfo {
    public:
        std::chrono::steady_clock::time_point time_processed;        
}