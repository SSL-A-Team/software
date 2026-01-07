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
#include "camera.hpp"
#include "filtered_robot.hpp"
#include "filtered_ball.hpp"
#include "measurements/ball_track.hpp"
#include "measurements/robot_track.hpp"

#include <rclcpp/rclcpp.hpp>
#include <ssl_league_msgs/msg/vision_wrapper.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include <vector>

namespace new_super_vision {

class VisionFilterNode : public rclcpp::Node
{
    // Important things this needs to do, which could be broken out into objects that this holds 
    // and uses when the node is ticked...

    // Check quality of stuff overall
    // Keep track of individual cameras - can be 1 - many
    // Fuse the info from those cameras into tracks
    // Output final prediction for robots and ball(s)

    public:
        explicit VisionFilterNode(const rclcpp::NodeOptions & options)
        : rclcpp::Node("new_ateam_vision_filter", options),
        game_controller_listener_(*this){
            ssl_vision_subscription_ = 
                create_subscription<ssl_league_msgs::msg::VisionWrapper>(
                std::string(Topics::kVisionMessages),
                10,
                std::bind(&VisionFilterNode::vision_callback, this, std::placeholders::_1)
            );

        }

    // Will also need to add publishers here... but want to determine whether we keep existing
    // approach that is in the old Vision Filter or do something else.
    
    private:
        // Subscribe to vision info from our processed messages
        // in ssl_vision_bridge_node.cpp
        std::map<int, Camera> cameras;

        std::vector<FilteredRobot> blue_robots;
        std::vector<FilteredRobot> yellow_robots;

        std::vector<BallTrack> ball_tracks;
        std::vector<RobotTrack> blue_tracks;
        std::vector<RobotTrack> yellow_tracks;

        void vision_callback(const ssl_league_msgs::msg::VisionWrapper::SharedPtr vision_wrapper_msg) {
            // Add detections to the queues
            if (!vision_wrapper_msg->detection.empty()){
                for (const auto& detection : vision_wrapper_msg->detection){
                    int detect_camera = detection.camera_id;
                    // Create a new camera if we haven't seen this one before
                    if (!(cameras.contains(detect_camera))){
                        cameras.try_emplace(detect_camera, detect_camera);
                    }

                    // Create a track from each robot in this message
                    for (const auto& bot : detection.yellow_robots) {
                        yellow_tracks.push_back(
                            RobotTrack(bot, detect_camera, ateam_common::TeamColor::Yellow)
                        ); 
                    }

                    for (const auto& bot : detection.blue_robots) {
                        blue_tracks.push_back(
                            RobotTrack(bot, detect_camera, ateam_common::TeamColor::Blue)
                        );
                    }

                    // Create a track from each ball in this message
                    for (const auto& ball: detection.balls) {
                        ball_tracks.push_back(
                            BallTrack(ball, detect_camera)
                        );
                    }
                }

                // Process all the new updates from the tracks
                // TODO (Christian) - Filter out known bad tracks first
                for (const auto& bot_track : blue_tracks){
                    
                }
                for (const auto& bot_track : yellow_tracks){

                }
                for (const auto& ball_track : ball_tracks){

                }
            }
            
            // // Add geometry to the cameras' msg queues
            // if (!vision_wrapper_msg->geometry.empty()){
            //     for (const auto& geometry: vision_wrapper_msg->geometry){
            //         for (const auto& calib : geometry.calibration){
            //             int geo_camera = calib.camera_id; 
            //             if (!(cameras.contains(geo_camera))){
            //                 cameras.try_emplace(geo_camera, geo_camera);
            //             }
            //         }
            //     }
            // }
            return; 
        }

        void timer_callback() {
            // Publish the updated vision info
            // Similar to the publish() method in TIGERs
            return;
        }

        ateam_common::GameControllerListener game_controller_listener_;
        rclcpp::Subscription<ssl_league_msgs::msg::VisionWrapper>::SharedPtr ssl_vision_subscription_;
};

} // namespace new_super_vision