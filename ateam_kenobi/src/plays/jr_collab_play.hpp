// Copyright 2026 A Team
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

#ifndef PLAYS__TEST_PLAYS__JR_COLLAB_PLAY_HPP_
#define PLAYS__TEST_PLAYS__JR_COLLAB_PLAY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "core/stp/play.hpp"
#include "skills/dribble.hpp"
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{
class JrCollabPlay : public stp::Play
{
public:
  static constexpr const char * kPlayName = "JrCollabPlay";

  explicit JrCollabPlay(stp::Options stp_options)
  : stp::Play(kPlayName, stp_options),
    jr_service_client_(getNode()->create_client<std_srvs::srv::Trigger>("/rcj_ble_node/play")),
    dribble_skill_(createChild<skills::Dribble>("dribble"))
  {
  }

  stp::PlayScore getScore(const World & world)
  {
    if(world.referee_info.running_command == ateam_common::GameCommand::ForceStart ||
      world.referee_info.running_command == ateam_common::GameCommand::Stop)
    {
      return stp::PlayScore::Max();
    } else {
      return stp::PlayScore::NaN();
    }
  }

  void enter() override
  {
    dribble_skill_.reset();
    have_sent_request_ = false;
  }

  void exit() override
  {

  }

  std::array<std::optional<RobotCommand>,
    16> runFrame(const World & world) override
  {
    if(world.referee_info.running_command == ateam_common::GameCommand::Stop &&
      world.referee_info.prev_command != ateam_common::GameCommand::Stop)
    {
      exit();
      enter();
    }
    std::array<std::optional<RobotCommand>, 16> commands;
    const auto available_bots = play_helpers::getAvailableRobots(world);
    if(available_bots.empty()) {
      return commands;
    }
    const auto robot = available_bots.front();
    if(world.referee_info.running_command == ateam_common::GameCommand::Stop) {
      const ateam_geometry::Point prep_pos{
        (-world.field.field_length / 2.0) + world.field.defense_area_depth + 1.0,
        -world.field.defense_area_width / 2.0
      };
      RobotCommand command;
      command.motion_intent = motion::intents::PositionFacing{
        .position = prep_pos,
        .face_target = world.ball.pos,
        .planner_options = {},
        .obstacles = {},
        .limits = motion::Limits{
          .linear_velocity = 2.0
        }
      };
      commands[robot.id] = command;
    }
    if(world.referee_info.running_command == ateam_common::GameCommand::ForceStart) {
      const ateam_geometry::Point target_ball_pos{
        (world.field.field_length / 2.0) - world.field.defense_area_depth - 0.5,
        0.0
      };
      dribble_skill_.setTarget(target_ball_pos);
      if(!dribble_skill_.isDone()) {
        commands[robot.id] = dribble_skill_.runFrame(world, robot);
        if(ateam_geometry::norm(world.ball.pos - target_ball_pos) > 2.0 && ateam_geometry::norm(world.ball.pos - robot.pos) < kRobotRadius + 0.05) {
          commands[robot.id]->kick = KickState::KickOnTouch;
          commands[robot.id]->kick_speed = 0.2;
        } else if(ateam_geometry::norm(world.ball.pos - target_ball_pos) > 1.0 && ateam_geometry::norm(world.ball.pos - robot.pos) < kRobotRadius + 0.05) {
          commands[robot.id]->kick = KickState::KickOnTouch;
          commands[robot.id]->kick_speed = 0.05;
        } else {
          commands[robot.id]->kick = KickState::Arm;
          commands[robot.id]->kick_speed = 0.0;
        }
        ForwardPlayInfo(dribble_skill_);
      } else {
        if(!have_sent_request_) {
          RCLCPP_INFO(getLogger(), "Calling JR play service.");
          client_future_ =
            jr_service_client_->async_send_request(
              std::make_shared<std_srvs::srv::Trigger::Request>()).future.share();
          have_sent_request_ = true;
        }
        const ateam_geometry::Point post_pos{
          target_ball_pos.x() - 0.5,
          -world.field.field_width / 4.0
        };
        RobotCommand command;
        command.motion_intent = motion::intents::PositionFacing{
          .position = post_pos,
          .face_target = world.ball.pos,
          .planner_options = {},
          .obstacles = {},
          .limits = motion::Limits{
            .linear_velocity = 2.0
          }
        };
        commands[robot.id] = command;
      }
    }
    return commands;
  }

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr jr_service_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture client_future_;
  skills::Dribble dribble_skill_;
  bool have_sent_request_ = false;

};
}  // namespace ateam_kenobi::plays
#endif  // PLAYS__TEST_PLAYS__JR_COLLAB_PLAY_HPP_
