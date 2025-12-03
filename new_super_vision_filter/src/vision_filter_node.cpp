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

#include <rclcpp/rclcpp.hpp>

#include "camera.hpp"

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
    // Subscribe to vision info from our processed messages
    // in ssl_vision_bridge_node.cpp
    ssl_vision_subscription_ =
      create_subscription<ssl_league_msgs::msg::VisionWrapper>(
      std::string(Topics::kVisionMessages),
      10,
      std::bind(&VisionFilterNode::vision_callback, this, std::placeholders::_1));

    // Will also need to add publishers here... but want to determine whether we keep existing
    // approach that is in the old Vision Filter or do something else.
    
    private:
        std::map<int, Camera> cameras;

        void vision_callback(const ssl_league_msgs::msg::VisionWrapper::SharedPtr vision_wrapper_msg) {
            // Add detections to the cameras' msg queues
            auto detection = vision_wrapper_msg->detection;
            // Create a new camera if we haven't seen this one before
            int detect_camera = detection->camera_id;
            if (!(cameras.contains(detect_camera))){
                cameras[detect_camera] = Camera(camera_id);
            }
            cameras[detect_camera].detection_queue.push_back(detection);
            
            // Add geometry to the cameras' msg queues
            auto geometry = vision_wrapper_msg->geometry;
            int geo_camera = geometry->camera_id; 
            if (!(cameras.contains(geo_camera))){
                cameras[geo_camera] = Camera(camera_id);
            }
            
        }

        void timer_callback() {
            // Publish the updated vision info
            // Similar to the publish() method in TIGERs
        }
}

} // namespace new_super_vision