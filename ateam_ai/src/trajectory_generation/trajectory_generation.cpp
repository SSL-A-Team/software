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

#include "trajectory_generation/trajectory_generation.hpp"

#include "trajectory_generation/trajectory_editor.hpp"
#include "trajectory_generation/trapezoidal_motion_profile.hpp"

namespace trajectory_generation
{
BehaviorPlan GetPlanFromGoal(
  BehaviorGoal behavior, int assigned_robot, const World & world)
{
  BehaviorPlan plan;

  switch (behavior.type) {
    case BehaviorGoal::Type::MovingKick:
    case BehaviorGoal::Type::PivotKick:
      // std::get<KickParam>(behavior.params).target_location;
      break;

    case BehaviorGoal::Type::OneTouchReceiveKick:
    case BehaviorGoal::Type::TwoTouchReceiveKick:
      // std::get<ReceiveParam>(behavior.params).receive_location;
      break;

    case BehaviorGoal::Type::Shot:
      break;

    case BehaviorGoal::Type::OneTouchShot:
      // std::get<ReceiveShotParam>(behavior.params).receive_location;
      break;

    case BehaviorGoal::Type::MoveToPoint:
      {
        Robot current_robot = world.plan_from_our_robots.at(assigned_robot).value();

        Eigen::Vector3d current, current_vel, target, target_vel;
        current.x() = current_robot.pos.x();
        current.y() = current_robot.pos.y();
        current.z() = 0;  // current_robot.theta;
        current_vel.x() = current_robot.vel.x();
        current_vel.y() = current_robot.vel.y();
        current_vel.z() = 0;  // current_robot.omega;

        target.x() = std::get<MoveParam>(behavior.params).target_location.x();
        target.y() = std::get<MoveParam>(behavior.params).target_location.y();
        target.z() = 0;
        target_vel.x() = 0;
        target_vel.y() = 0;
        target_vel.z() = 0;
        Eigen::Vector3d max_vel{2, 2, 0.5};  // TODO(jneiger): Set as params
        Eigen::Vector3d max_accel{2, 2, 0.5};
        double dt = 0.01;  // TODO(jneiger): Feed this down from above
        Trajectory trajectory = TrapezoidalMotionProfile::Generate3d(
          current, current_vel, target,
          target_vel, max_vel, max_accel,
          dt, world.current_time + world.immutable_duration);

        plan.trajectory = trajectory;
      }
      break;

    case BehaviorGoal::Type::CostFunctionPoint:
      break;

    default:
      break;
  }

  plan.assigned_robot_id = assigned_robot;
  return plan;
}
}  // namespace trajectory_generation
