// Copyright 2021 A Team
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


#ifndef CORE__TYPES__REFEREE_INFO_HPP_
#define CORE__TYPES__REFEREE_INFO_HPP_

#include <chrono>
#include <optional>
#include <ateam_common/game_controller_listener.hpp>
#include <ateam_geometry/ateam_geometry.hpp>

namespace ateam_kenobi
{
struct RefereeInfo
{
  int our_goalie_id = -1;
  int their_goalie_id = -1;
  ateam_common::GameStage current_game_stage = ateam_common::GameStage::Unknown;
  ateam_common::GameCommand running_command = ateam_common::GameCommand::Halt;
  ateam_common::GameCommand prev_command = ateam_common::GameCommand::Halt;
  std::optional<ateam_geometry::Point> designated_position;
  std::chrono::system_clock::time_point command_time;
};
}  // namespace ateam_kenobi

#endif  // CORE__TYPES__REFEREE_INFO_HPP_
