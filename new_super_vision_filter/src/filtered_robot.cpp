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

#include "filtered_robot.hpp"
#include "filter_types.hpp"
#include "measurements/robot_track.hpp"

#include <cmath>
#include <chrono>
#include <Eigen/Core>
#include <ateam_geometry/ateam_geometry.hpp>

// See https://thekalmanfilter.com/extended-kalman-filter-python-example/
// or https://thekalmanfilter.com/kalman-filter-explained-simply/
// OR https://github.com/mherb/kalman/blob/master/examples/Robot1/main.cpp

FilteredRobot::FilteredRobot(const RobotTrack & track, ateam_common::TeamColor & team_color)
: posFilterXY(), posFilterW(), bot_id(track.getId()), team(team_color)
{
        // Initialize XY KF
  PosState initial_state_xy;
  initial_state_xy <<
    track.pos.x(),
    track.pos.y(),
    0,
    0;
  posFilterXY.init(initial_state_xy);
  Kalman::Matrix<double, 4, 4> xy_covariance;
        // This is in m, so initial covariance is 100 mm.
        // We don't get a velocity input in the measurement itself,
        // so that has a large initial uncertainty.
  xy_covariance << 1e-2, 0, 0, 0,
    0, 1e-2, 0, 0,
    0, 0, 1e3, 0,
    0, 0, 0, 1e3;
  posFilterXY.setCovariance(xy_covariance);

        // Initialize angular KF
        /*
            State vector is simply
            w_pos,
            w_vel
        */
  AngleState initial_state_w;
  initial_state_w <<
    track.angle.w(),
    0;
  posFilterW.init(initial_state_w);
        /*
            Initial covariance is approx 2 deg. for pos,
            10 deg. for vel
        */
  Kalman::Matrix<double, 2, 2> w_covariance;
  w_covariance << std::pow(2 * M_PI / 180.0, 2), 0,
    0, std::pow(10 * M_PI / 180.0, 2);
  posFilterW.setCovariance(w_covariance);
}

void FilteredRobot::update(const RobotTrack & track)
{
    // Make sure this detection isn't crazy off from our previous ones
    // (unless our filter is still new/only has a few measurements)
  if (age < oldEnough) {
    ++age;
  }
  if (health < maxHealth) {
    health += 2;
  }
  bool is_new = age < oldEnough;
    // As long as its reasonable, update the Kalman Filter
  const std::chrono::time_point<std::chrono::steady_clock> now =
    std::chrono::steady_clock::now();
    // If it's been too long, don't use this message
  if (now - track.getTimestamp() > update_threshold || is_new) {
    return;
  }
    // Predict state forward
    // Predict covariance forward
    // (All encompassed by the .predict() function)
  auto xy_pred = posFilterXY.predict(systemModelXY);
  auto w_pred = posFilterW.predict(systemModelW);
    // Update the Jacobian matrix (contained in filter)
    // Compute Kalman gain (contained in filter)
    // Update state estimate (returned)
    // Update covariance estimate (contained in filter)
    // All encompassed by the .update() function
  posXYEstimate = posFilterXY.update(measurementModelXY, track.pos);
  posWEstimate = posFilterW.update(measurementModelW, track.angle);
}

ateam_msgs::msg::VisionStateRobot FilteredRobot::toMsg()
{
  ateam_msgs::msg::VisionStateRobot robot_state_msg{};
  bool is_new = age < oldEnough;

  if (health > 0 && !is_new) {
    robot_state_msg.visible = true;
    robot_state_msg.pose.position.x = posXYEstimate.px();
    robot_state_msg.pose.position.y = posXYEstimate.py();
    robot_state_msg.twist.linear.x = posXYEstimate.vx();
    robot_state_msg.twist.linear.y = posXYEstimate.vy();

    robot_state_msg.pose.orientation.x = 0;
    robot_state_msg.pose.orientation.y = 0;
    robot_state_msg.pose.orientation.z = 1;
    robot_state_msg.pose.orientation.w = posWEstimate.pw();
    robot_state_msg.twist.angular.z = posWEstimate.vw();

        // Convert to body velocities for plotting/debugging
    ateam_geometry::Vector velocity(robot_state_msg.twist.linear.x, robot_state_msg.twist.linear.y);
    CGAL::Aff_transformation_2<ateam_geometry::Kernel> transformation(CGAL::ROTATION,
      std::sin(-posWEstimate.pw()), std::cos(-posWEstimate.pw()));
    const auto velocity_trans = velocity.transform(transformation);
    robot_state_msg.twist_body.linear.x = velocity_trans.x();
    robot_state_msg.twist_body.linear.y = velocity_trans.y();
    robot_state_msg.twist_body.angular.z = robot_state_msg.twist.angular.z;
    --health;
  }
  return robot_state_msg;
}

int FilteredRobot::getId() const
{
  return bot_id;
}

bool FilteredRobot::isHealthy() const
{
  return health > 0;
}
