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
#include <rclcpp_components/register_node_macro.hpp>
#include <ssl_league_msgs/msg/vision_wrapper.hpp>
#include <ateam_msgs/msg/field_info.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <vector>
#include <algorithm>
#include <optional>

using namespace std::chrono_literals;

namespace new_super_vision
{

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
    game_controller_listener_(*this)
  {
    ssl_vision_subscription_ =
      create_subscription<ssl_league_msgs::msg::VisionWrapper>(
                std::string(Topics::kVisionMessages),
                10,
                std::bind(&VisionFilterNode::vision_callback, this, std::placeholders::_1)
      );

    timer_ = create_wall_timer(10ms, std::bind(&VisionFilterNode::timer_callback, this));

    ball_publisher_ = create_publisher<ateam_msgs::msg::VisionStateBall>(
                std::string(Topics::kBall),
                rclcpp::SystemDefaultsQoS());


    ateam_common::indexed_topic_helpers::create_indexed_publishers
    <ateam_msgs::msg::VisionStateRobot>(
            blue_robots_publisher_,
            Topics::kBlueTeamRobotPrefix,
            rclcpp::SystemDefaultsQoS(),
            this
    );
    ateam_common::indexed_topic_helpers::create_indexed_publishers
    <ateam_msgs::msg::VisionStateRobot>(
            yellow_robots_publisher_,
            Topics::kYellowTeamRobotPrefix,
            rclcpp::SystemDefaultsQoS(),
            this
    );

    ssl_vision_subscription_ =
      create_subscription<ssl_league_msgs::msg::VisionWrapper>(
                std::string(Topics::kVisionMessages),
                10,
                std::bind(&VisionFilterNode::vision_callback, this, std::placeholders::_1));

    field_subscription_ =
      create_subscription<ateam_msgs::msg::FieldInfo>(
                std::string(Topics::kField),
                10,
                std::bind(&VisionFilterNode::field_callback, this, std::placeholders::_1));
  }

    // Will also need to add publishers here and wall clock timer

private:
  std::map<int, Camera> cameras;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<FilteredRobot> blue_robots;
  std::vector<FilteredRobot> yellow_robots;
  std::optional<FilteredBall> ball;

  std::vector<BallTrack> ball_tracks;
  std::vector<RobotTrack> blue_tracks;
  std::vector<RobotTrack> yellow_tracks;

        // All this stuff interacts with other nodes (pub/sub related)
  std::array<rclcpp::Publisher<ateam_msgs::msg::VisionStateRobot>::SharedPtr,
    16> blue_robots_publisher_;
  std::array<rclcpp::Publisher<ateam_msgs::msg::VisionStateRobot>::SharedPtr,
    16> yellow_robots_publisher_;
  rclcpp::Publisher<ateam_msgs::msg::VisionStateBall>::SharedPtr ball_publisher_;

  ateam_common::GameControllerListener game_controller_listener_;
  rclcpp::Subscription<ssl_league_msgs::msg::VisionWrapper>::SharedPtr ssl_vision_subscription_;
        // The two below are in case we are sharing a field during testing at event
  rclcpp::Subscription<ateam_msgs::msg::FieldInfo>::SharedPtr field_subscription_;
  int ignore_side_;

  void vision_callback(const ssl_league_msgs::msg::VisionWrapper::SharedPtr vision_wrapper_msg)
  {
            // Add detections to the queues
    if (!vision_wrapper_msg->detection.empty()) {
      for (const auto & detection : vision_wrapper_msg->detection) {
        int detect_camera = detection.camera_id;
                    // Create a new camera if we haven't seen this one before
        if (!(cameras.contains(detect_camera))) {
          cameras.try_emplace(detect_camera, detect_camera);
        }

                    // Create a track from each robot in this message
        for (const auto & bot : detection.robots_yellow) {
          ateam_common::TeamColor team_color = ateam_common::TeamColor::Yellow;
          yellow_tracks.push_back(
                            RobotTrack(bot, detect_camera, team_color)
          );
        }

        for (const auto & bot : detection.robots_blue) {
          ateam_common::TeamColor team_color = ateam_common::TeamColor::Blue;
          blue_tracks.push_back(
                            RobotTrack(bot, detect_camera, team_color)
          );
        }

                    // Create a track from each ball in this message
        for (const auto & ball: detection.balls) {
          ball_tracks.push_back(
                            BallTrack(ball, detect_camera)
          );
        }
      }

                // Sort our tracks to make sure they are in order of the time received
                // in case created/processed them out of order

                // Process all the new updates from the tracks
      for (const auto & bot_track : blue_tracks) {
        ateam_common::TeamColor team_color = ateam_common::TeamColor::Blue;
        auto it = std::find_if(
                        blue_robots.begin(),
                        blue_robots.end(),
          [bot_track](const FilteredRobot & bot){return bot_track.getId() == bot.getId();}
        );
        if (it != blue_robots.end()) {
          blue_robots.push_back(
                            FilteredRobot(bot_track, team_color)
          );
        } else {
          it->update(bot_track);
        }
      }
      for (const auto & bot_track : yellow_tracks) {
        ateam_common::TeamColor team_color = ateam_common::TeamColor::Yellow;
        auto it = std::find_if(
                        yellow_robots.begin(),
                        yellow_robots.end(),
          [bot_track](const FilteredRobot & bot){return bot_track.getId() == bot.getId();}
        );
        if (it != yellow_robots.end()) {
          yellow_robots.push_back(
                            FilteredRobot(bot_track, team_color)
          );
        } else {
          it->update(bot_track);
        }
      }
      for (const auto & ball_track : ball_tracks) {
        if (!ball.has_value()) {
          ball = FilteredBall(ball_track);
        } else {
          ball->update(ball_track);
        }
      }
    }

    return;
  }

  void timer_callback()
  {
            // Remove the ball if it's bad quality
    if (ball.has_value()) {
      if (!ball.value().isHealthy()) {
        ball.reset();
      }
    }
            // Erase any robots that are bad quality
    std::erase_if(blue_robots, [](FilteredRobot & bot) {return !bot.isHealthy();});
    std::erase_if(yellow_robots, [](FilteredRobot & bot) {return !bot.isHealthy();});
            // Publish the most recent ball
    ateam_msgs::msg::VisionStateBall ball_msg{};
    if (ball.has_value()) {
      ball_msg = ball.value().toMsg();
    }
    ball_publisher_->publish(ball_msg);

            // Publish the robots
            // - Blue -
    for (std::size_t id = 0; id < 16; id++) {
      auto robot_msg = ateam_msgs::msg::VisionStateRobot{};
      auto it = std::find_if(
                    blue_robots.begin(),
                    blue_robots.end(),
        [id](const FilteredRobot & bot){return id == bot.getId();}
      );
      if (it != blue_robots.end()) {
        robot_msg = it->toMsg();
      }
      blue_robots_publisher_.at(id)->publish(robot_msg);
    }
            // - Yellow -
    for (std::size_t id = 0; id < 16; id++) {
      auto robot_msg = ateam_msgs::msg::VisionStateRobot{};
      auto it = std::find_if(
                    yellow_robots.begin(),
                    yellow_robots.end(),
        [id](const FilteredRobot & bot){return id == bot.getId();}
      );
      if (it != yellow_robots.end()) {
        robot_msg = it->toMsg();
      }
      yellow_robots_publisher_.at(id)->publish(robot_msg);
    }
    return;
  }

  void field_callback(
    const ateam_msgs::msg::FieldInfo::SharedPtr field_info_msg)
  {
    const auto team_side = game_controller_listener_.GetTeamSide();
    if (team_side == ateam_common::TeamSide::PositiveHalf) {
      ignore_side_ = -field_info_msg->ignore_side;
    } else {
      ignore_side_ = field_info_msg->ignore_side;
    }
  }
};

} // namespace new_super_vision

RCLCPP_COMPONENTS_REGISTER_NODE(new_super_vision::VisionFilterNode)
