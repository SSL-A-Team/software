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


#ifndef SKILLS__CAPTURE_HPP_
#define SKILLS__CAPTURE_HPP_

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "core/play_helpers/easy_move_to.hpp"
#include "core/play_helpers/intercept_calculation.hpp"
#include <ateam_common/robot_constants.hpp>
#include "core/stp/skill.hpp"
#include "core/types/world.hpp"


namespace ateam_kenobi::skills
{

class Capture : public stp::Skill
{
public:
  explicit Capture(stp::Options stp_options);

  void Reset();

  ateam_geometry::Point getAssignmentPoint(const World & world)
  {
    return world.ball.pos;
  }

  bool isDone()
  {
    return done_;
  }


  ateam_msgs::msg::RobotMotionCommand runFrame(const World & world, const Robot & robot);

  /**
 * @brief Set the default obstacles planner option on the internal EasyMoveTo
 */
  void SetUseDefaultObstacles(bool use_obstacles)
  {
    path_planning::PlannerOptions options = easy_move_to_.getPlannerOptions();
    options.use_default_obstacles = use_obstacles;
    easy_move_to_.setPlannerOptions(options);
  }

  /**
   * @brief Set the capture speed used to approach the ball in the final phase
   *
   * @param speed Speed in meters per second
   */
  void SetCaptureSpeed(double speed)
  {
    capture_speed_ = speed;
  }

  /**
   * @brief Set if capture should attempt to steal the ball from an opponent robot
   *
   * @param should_steal Whether or not capture should attemp to steal the ball from opposing robots
   */
  void SetShouldSteal(double should_steal)
  {
    steal_from_opponent_ = should_steal;
  }

private:
  play_helpers::EasyMoveTo easy_move_to_;
  play_helpers::InterceptResults intercept_result_;
  bool done_ = false;

  // Sets if the robot should go to the approach point and slow down before
  // entering the capture state even if it can directly approach the ball
  bool always_approach_first_ = true;
  // TODO: actually listen to this boolean
  bool steal_from_opponent_ = true;

  int ball_detected_filter_ = 0;
  double approach_radius_ = kRobotRadius + kBallRadius + 0.15;  // m

  double capture_speed_ = 2.0;  // m/s

  double max_speed_ = 2.0;  // m/s
  double extract_accel_limit_ = 1.0;  // for extracting ball m/s/s
  double capture_decel_limit_ = 0.5;  // for approaching ball m/s/s

  // Ball should have enough room for the robot to fit between it and the obstacle with a bit of extra space
  double ball_distance_from_obstacle_ = kRobotDiameter + kBallRadius + 0.03;

  ateam_geometry::Point approach_point;

  enum class State
  {
    Intercept,
    MoveToApproachPoint,
    Capture,
    Extract
  };
  State state_ = State::MoveToApproachPoint;

  ateam_geometry::Point calculateApproachPoint(const World & world, const Robot & robot);
  ateam_geometry::Point calculateInterceptPoint(const World & world, const Robot & robot);
  std::optional<ateam_geometry::Vector> calculateObstacleOffset(const World & world);

  bool isBallNearInsideGoal(const World & world, double safe_distance);
  bool isBallNearOutsideGoal(const World & world, double safe_distance);
  bool isBallNearWall(const World & world, double safe_distance);
  bool filteredBallSense(const Robot & robot);

  void chooseState(const World & world, const Robot & robot);

  ateam_msgs::msg::RobotMotionCommand runIntercept(const World & world, const Robot & robot);
  ateam_msgs::msg::RobotMotionCommand runMoveToApproachPoint(const World & world, const Robot & robot);
  ateam_msgs::msg::RobotMotionCommand runExtract(const World & world, const Robot & robot);
  ateam_msgs::msg::RobotMotionCommand runCapture(const World & world, const Robot & robot);
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__CAPTURE_HPP_
