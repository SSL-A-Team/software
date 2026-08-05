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

#ifndef FILTER_TYPES_HPP_
#define FILTER_TYPES_HPP_

#include <chrono>
#include <cmath>
#include <Eigen/Core>

namespace ateam_super_vision
{
/*
    Position measurement for robot or balls

    Structure is {
        x_pos
        y_pos
    }

    Measurements are in m
*/
class PosMeasurement : public Eigen::Vector2d
{
public:
  using Base = Eigen::Vector2d;
  using Base::Base;
  using Base::operator=;

  static constexpr size_t X = 0;
  static constexpr size_t Y = 1;

  double x() const
  {
    return (*this)[X];
  }

  double y() const
  {
    return (*this)[Y];
  }
};

/*
    4D state vector for robot or ball position

    State vector structure is
        { x_pos,
        y_pos,
        x_vel,
        y_vel }

    Measurements are in m
*/
class PosState : public Eigen::Vector4d
{
public:
  using Base = Eigen::Vector4d;
  using Base::Base;
  using Base::operator=;

  static constexpr size_t PX = 0;
  static constexpr size_t PY = 1;
  static constexpr size_t VX = 2;
  static constexpr size_t VY = 3;

  double px() const {return (*this)[PX];}
  double py() const {return (*this)[PY];}
  double vx() const {return (*this)[VX];}
  double vy() const {return (*this)[VY];}
};

/*
    Measurement model for robots/balls' X and Y pos from position
    measurements.

    Curently assumes no measurement noise.
*/
class PosMeasurementModel
{
public:
  // H matrix: maps a PosState to the position-only PosMeasurement it predicts.
  static Eigen::MatrixXd measurementMatrix()
  {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 4);
    H(PosMeasurement::X, PosState::PX) = 1;     // dz_px / d_px
    H(PosMeasurement::Y, PosState::PY) = 1;     // dz_py / d_py
    return H;
  }

  // R matrix. Currently assumes no measurement noise.
  static Eigen::MatrixXd measurementNoiseCovar()
  {
    return Eigen::MatrixXd::Zero(2, 2);
  }
};

/*
    System (measurement to state) transition model for X/Y position
    that assumes a constant velocity and no control inputs.
*/
class PosSystemModel
{
public:
  PosSystemModel()
  : last_update(std::chrono::steady_clock::now()) {}

  // Seconds elapsed since the previous call, resetting the internal clock.
  double stepDt()
  {
    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> dt = now - last_update;
    last_update = now;
    return dt.count();
  }

  /*
      F matrix for the given elapsed time.

      We assume the velocity stays constant and add v * dt to the
      current position.

      pos_t = pos_{t-1} + vel_{t-1} * dt
      vel_t = vel_{t-1}
  */
  static Eigen::MatrixXd stateTransition(double dt_s)
  {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(4, 4);
    F(PosState::PX, PosState::VX) = dt_s;
    F(PosState::PY, PosState::VY) = dt_s;
    return F;
  }

  // Q matrix. Values borrowed from the previous vision filter.
  static Eigen::MatrixXd processNoiseCovar()
  {
    return Eigen::MatrixXd::Identity(4, 4) * 1e-3;
  }

private:
  std::chrono::time_point<std::chrono::steady_clock> last_update;
};

/*
    Angular position measurement (in rad)
*/
class AngleMeasurement : public Eigen::Matrix<double, 1, 1>
{
public:
  using Base = Eigen::Matrix<double, 1, 1>;
  using Base::Base;
  using Base::operator=;

  static constexpr size_t W = 0;

  double w() const
  {
    return (*this)[W];
  }
};

/*
    Angular state (position and velocity)

    State vector format is
    {
        w_pos
        w_vel
    }
*/
class AngleState : public Eigen::Vector2d
{
public:
  using Base = Eigen::Vector2d;
  using Base::Base;
  using Base::operator=;

  static constexpr size_t PW = 0;
  static constexpr size_t VW = 1;

  double pw() const {return (*this)[PW];}
  double vw() const {return (*this)[VW];}
};

/*
    Angle measurement model.

    Currently assumes no measurement noise.
*/
class AngleMeasurementModel
{
public:
  // H matrix: maps an AngleState to the angle-only AngleMeasurement it predicts.
  static Eigen::MatrixXd measurementMatrix()
  {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 2);
    H(AngleMeasurement::W, AngleState::PW) = 1;     // dz_pw / d_pw
    return H;
  }

  // R matrix. Measurement error borrowed from previous vision filter.
  static Eigen::MatrixXd measurementNoiseCovar()
  {
    const double sigma_theta_squared = 0.01;     // Position measurement error
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(1, 1);
    R(0, 0) = sigma_theta_squared;
    return R;
  }
};

/*
    System (measurement to state) transition model for angular position
    that assumes a constant angular velocity and no control inputs.
*/
class AngleSystemModel
{
public:
  AngleSystemModel()
  : last_update(std::chrono::steady_clock::now()) {}

  // Seconds elapsed since the previous call, resetting the internal clock.
  double stepDt()
  {
    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> dt = now - last_update;
    last_update = now;
    return dt.count();
  }

  /*
      F matrix for the given elapsed time.

      We assume the angular velocity stays constant and add w * dt to the
      current angle. Callers are responsible for wrapping the resulting
      angle to (-pi, pi], since that's a nonlinear operation the F matrix
      can't express.

      pos_t = pos_{t-1} + vel_{t-1} * dt
      vel_t = vel_{t-1}
  */
  static Eigen::MatrixXd stateTransition(double dt_s)
  {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(2, 2);
    F(AngleState::PW, AngleState::VW) = dt_s;
    return F;
  }

  static double wrapAngle(double angle)
  {
    return std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
  }

  // Q matrix. No equivalent value was defined in the previous vision
  // filter; this magnitude matches the position filter's process noise.
  static Eigen::MatrixXd processNoiseCovar()
  {
    return Eigen::MatrixXd::Identity(2, 2) * 1e-3;
  }

private:
  std::chrono::time_point<std::chrono::steady_clock> last_update;
};
} // namespace ateam_super_vision

#endif // FILTER_TYPES_HPP
