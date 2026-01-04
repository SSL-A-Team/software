#ifndef FRAME_INFO_HPP_ 
#define FRAME_INFO_HPP_ 

#include <chrono>

class LastFrameInfo {
    public:
        std::chrono::steady_clock::time_point time_processed;        
};

#endif // FRAME_INFO_HPP_ 
