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

#include <algorithm>

#include "camera.hpp"
#include "filtered_ball.hpp"
#include "filtered_robot.hpp"

Camera::Camera(int camera_id) : camera_id(camera_id) {
    public:   
        // Need to process an individual frame
        // Set geometry from VisionGeometryCameraCalibration.msg
        // Have a queue/buffer that we can remove old frames/have a set capacity

        void process_detection_frame(const ssl_league_msgs::msg::VisionDetectionFrame detection_frame_msg){
            process_balls(detection_frame_msg);
            process_robots(detection_frame_msg);
        }

        void process_camera_geometry(const ssl_league_msgs::msg::VisionGeometryData){};

        void clear_old_messages();

        void process_balls(const ssl_league_msgs::msg::VisionDetectionFrame detection_frame_msg){
            // Geometry msgs stuff...
            // https://docs.ros2.org/foxy/api/geometry_msgs/index-msg.html
            for (auto ball : detection_frame_msg->balls) {
                // For all of our balls, see if we think this is close to an existing measurement
                // If not, create a new one
                tracked_balls.push_back(FilteredBall(ball));
            }
        }

        void process_robots(const ssl_league_msgs::msg::VisionDetectionFrame detection_frame_msg){
            for (auto robot_detection : detection_frame_msg->robots_blue){
                // Check if this is close to an existing measurement (from an id that we have?)
                auto existing_bot = std::find_if(tracked_robots.begin(), tracked_robots.end(), [](const FilteredRobot& bot) {
                    return bot.bot_id == robot_detection->robot_id; 
                });
                if (existing_bot != tracked_robots.end()){
                    existing_bot.update(robot_detection);
                } else {
                    // If not, create a new one
                    tracked_robots.push_back(FilteredRobot(robot_detection, 0));
                }
            }
            for (auto robot_detection : detection_frame_msg->robots_yellow){
                // Check if this is close to an existing measurement (from an id that we have?)
                auto existing_bot = std::find_if(tracked_robots.begin(), tracked_robots.end(), [](const FilteredRobot& bot) {
                    return bot.bot_id == robot_detection->robot_id; 
                });
                if (existing_bot != tracked_robots.end()){
                    existing_bot.update(robot_detection);
                } else {
                    // If not, create a new one
                    tracked_robots.push_back(FilteredRobot(robot_detection, 1));
                }
            }
        }

        void create_new_ball_track();

        void create_new_robot_track();
    
    private:
        int camera_id;
        LastFrameInfo last_processed_frame;
        std::vector<FilteredBall> tracked_balls;
        std::vector<FilteredRobot> tracked_robots;
}