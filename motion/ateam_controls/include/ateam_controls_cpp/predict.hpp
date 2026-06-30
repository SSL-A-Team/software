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

#ifndef ATEAM_CONTROLS_CPP__PREDICT_HPP_
#define ATEAM_CONTROLS_CPP__PREDICT_HPP_

#include <ateam_game_state/robot.hpp>
#include <ateam_geometry/types.hpp>

namespace ateam_controls_cpp::predict
{

namespace modes
{
struct Off{};

struct EStopBrake{};

struct GlobalPosition
{
  float global_x;
  float global_y;
  float global_theta;
  float max_linear_vel;
  float max_angular_vel;
  float max_linear_acc;
  float max_angular_acc;
};

struct GlobalVelocity{};

struct LocalVelocity{};

struct GlobalAccel{};

struct LocalAccel{};

struct HeadingPivot{};

struct PointPivot{};

struct HeadingLine{};

struct PointLine{};
}  // namespace modes

using ateam_game_state::Robot;

ateam_geometry::Point PositionAtT(const Robot & robot, const modes::Off & params, const double t);
ateam_geometry::Point PositionAtT(const Robot & robot, const modes::EStopBrake & params, const double t);
ateam_geometry::Point PositionAtT(const Robot & robot, const modes::GlobalPosition & params, const double t);
ateam_geometry::Point PositionAtT(const Robot & robot, const modes::GlobalVelocity & params, const double t);
ateam_geometry::Point PositionAtT(const Robot & robot, const modes::LocalVelocity & params, const double t);
ateam_geometry::Point PositionAtT(const Robot & robot, const modes::GlobalAccel & params, const double t);
ateam_geometry::Point PositionAtT(const Robot & robot, const modes::LocalAccel & params, const double t);
ateam_geometry::Point PositionAtT(const Robot & robot, const modes::HeadingPivot & params, const double t);
ateam_geometry::Point PositionAtT(const Robot & robot, const modes::PointPivot & params, const double t);
ateam_geometry::Point PositionAtT(const Robot & robot, const modes::HeadingLine & params, const double t);
ateam_geometry::Point PositionAtT(const Robot & robot, const modes::PointLine & params, const double t);

}  // ateam_controls_cpp::predict

#endif  // ATEAM_CONTROLS_CPP__PREDICT_HPP_
