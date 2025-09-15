// Copyright 2025 A Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#include <deque>
#include <ssl_league_msgs/msg/vision_geometry_data.hpp>

class Camera {
    // Need to process an individual frame
    // Set geometry from VisionGeometryCameraCalibration.msg
    // Have a queue/buffer that we can remove old frames/have a set capacity
    void process_detection_frame(){}

    void process_camera_geometry(){}

    void clear_old_messages(){}

    void process_balls(){}

    void process_robots(){}

    void create_new_ball_track(){}

    void create_new_robot_track(){}

    private:
        std::deque<ssl_league_msgs::msg::VisionDetectionFrame> detection_queue;
        std::deque<ssl_league_msgs::msg::VisionGeometryData> geometry_queue;
}