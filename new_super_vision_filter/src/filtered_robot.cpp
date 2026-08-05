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
#include "measurements/robot_measurement.hpp"

#include <cmath>
#include <chrono>
#include <Eigen/Core>
#include <ateam_geometry/ateam_geometry.hpp>
#include <iostream>

// See https://thekalmanfilter.com/extended-kalman-filter-python-example/
// or https://thekalmanfilter.com/kalman-filter-explained-simply/
// OR https://github.com/mherb/kalman/blob/master/examples/Robot1/main.cpp

namespace ateam_super_vision {
FilteredRobot::FilteredRobot(
  const RobotMeasurement & measurement,
  ateam_common::TeamColor & team_color)
: posFilterXY(), posFilterW(), bot_id(measurement.getId()), team(team_color)
{
        // Initialize XY KF
  Eigen::VectorXd initial_state_xy(4);
  initial_state_xy <<
    measurement.pos.x(),
    measurement.pos.y(),
    0,
    0;
        // This is in m, so initial covariance is 100 mm.
        // We don't get a velocity input in the measurement itself,
        // so that has a large initial uncertainty.
  Eigen::MatrixXd xy_covariance(4, 4);
  xy_covariance << 1e-4, 0, 0, 0,
    0, 1e-4, 0, 0,
    0, 0, 1e-2, 0,
    0, 0, 0, 1e-2;

  Eigen::MatrixXd control_model_xy = Eigen::MatrixXd::Zero(4, 0);
  Eigen::MatrixXd measurement_model_xy = PosMeasurementModel::measurementMatrix();
  Eigen::MatrixXd measurement_noise_covar_xy = PosMeasurementModel::measurementNoiseCovar();
  Eigen::MatrixXd process_noise_covar_xy = PosSystemModel::processNoiseCovar();

  posFilterXY.set_control_model(control_model_xy);
  posFilterXY.set_measurement_model(measurement_model_xy);
  posFilterXY.set_measurement_noise_covar(measurement_noise_covar_xy);
  posFilterXY.set_process_noise_covar(process_noise_covar_xy);

  posFilterXY.init(initial_state_xy, xy_covariance);
  posXYEstimate = initial_state_xy;

        // Initialize angular KF
        /*
            State vector is simply
            w_pos,
            w_vel
        */
  Eigen::VectorXd initial_state_w(2);
  initial_state_w <<
    measurement.angle.w(),
    0;
        /*
            Initial covariance is approx 2 deg. for pos,
            10 deg. for vel
        */
  Eigen::MatrixXd w_covariance(2, 2);
  w_covariance << M_PI / 180.0, 0,
    0, M_PI / 180.0;

  Eigen::MatrixXd control_model_w = Eigen::MatrixXd::Zero(2, 0);
  Eigen::MatrixXd measurement_model_w = AngleMeasurementModel::measurementMatrix();
  Eigen::MatrixXd measurement_noise_covar_w = AngleMeasurementModel::measurementNoiseCovar();
  Eigen::MatrixXd process_noise_covar_w = AngleSystemModel::processNoiseCovar();

  posFilterW.set_control_model(control_model_w);
  posFilterW.set_measurement_model(measurement_model_w);
  posFilterW.set_measurement_noise_covar(measurement_noise_covar_w);
  posFilterW.set_process_noise_covar(process_noise_covar_w);

  posFilterW.init(initial_state_w, w_covariance);
  posWEstimate = initial_state_w;
}

void FilteredRobot::update(const RobotMeasurement & measurement)
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
  if (now - measurement.getTimestamp() > update_threshold) {
    return;
  }
  if (is_new) {
    return;
  }
    // The state transition matrices (F) depend on elapsed time, so they're
    // rebuilt every update with the latest dt.
  Eigen::MatrixXd state_transition_xy = PosSystemModel::stateTransition(systemModelXY.stepDt());
  posFilterXY.set_state_transition_model(state_transition_xy);
  Eigen::MatrixXd state_transition_w = AngleSystemModel::stateTransition(systemModelW.stepDt());
  posFilterW.set_state_transition_model(state_transition_w);

    // Predict state forward
    // Predict covariance forward
    // (All encompassed by the .predict() function)
  Eigen::VectorXd control_input_xy(0);
  posFilterXY.predict(control_input_xy);
  Eigen::VectorXd control_input_w(0);
  posFilterW.predict(control_input_w);

    // Compute Kalman gain (contained in filter)
    // Update state estimate (contained in filter)
    // Update covariance estimate (contained in filter)
    // All encompassed by the .update() function
  Eigen::VectorXd z_xy(2);
  z_xy << measurement.pos.x(), measurement.pos.y();
  posFilterXY.update(z_xy);
  posXYEstimate = posFilterXY.get_state_estimate();

  Eigen::VectorXd z_w(1);
  z_w << measurement.angle.w();
  posFilterW.update(z_w);
  posWEstimate = posFilterW.get_state_estimate();
  posWEstimate[AngleState::PW] = AngleSystemModel::wrapAngle(posWEstimate[AngleState::PW]);
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

    robot_state_msg.pose.orientation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), posWEstimate.pw()));
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
} // namespace ateam_super_vision
