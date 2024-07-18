// Copyright 2024 A Team
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

#ifndef TACTICS__MULTI_IDLE_HPP_
#define TACTICS__MULTI_IDLE_HPP_

#include "stp/tactic.hpp"
#include "play_helpers/easy_move_to.hpp"
#include "motion/motion_controller.hpp"

namespace ateam_kenobi::tactics
{

class MultiMoveTo : public stp::Tactic
{
public:
  explicit MultiMoveTo(stp::Options stp_options);

  void Reset();

  std::vector<ateam_geometry::Point> GetAssignmentPoints();

  void RunFrame(
    const World & world,
    const std::vector<std::optional<Robot>> & robots,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & motion_commands);

  void SetTargetPoints(const std::vector<ateam_geometry::Point> & targets)
  {
    target_points_ = targets;
  }

  void SetFaceNone()
  {
    angle_mode_ = AngleMode::no_face;
  }

  void SetFaceTravel()
  {
    angle_mode_ = AngleMode::face_travel;
  }

  void SetFaceAbsolue(double angle)
  {
    angle_mode_ = AngleMode::face_absolute;
    absolute_angle_reference_ = angle;
  }

  void SetFacePoint(const ateam_geometry::Point & point)
  {
    angle_mode_ = AngleMode::face_point;
    point_angle_reference_ = point;
  }

  void SetObstacles(const std::vector<ateam_geometry::AnyShape> & obstacles)
  {
    obstacles_ = obstacles;
  }

  void ClearObstacles()
  {
    obstacles_.clear();
  }

private:
  AngleMode angle_mode_ = AngleMode::no_face;
  double absolute_angle_reference_ = 0.0;
  ateam_geometry::Point point_angle_reference_;

  std::vector<ateam_geometry::AnyShape> obstacles_;

  std::vector<ateam_geometry::Point> target_points_;

  std::array<play_helpers::EasyMoveTo, 16> easy_move_tos_;
};

}  // namespace ateam_kenobi::tactics

#endif  // TACTICS__MULTI_IDLE_HPP_
