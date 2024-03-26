#ifndef PLAYS__WAYPOINTS_PLAY_HPP_
#define PLAYS__WAYPOINTS_PLAY_HPP_

#include <array>
#include <chrono>
#include <vector>
#include <string>
#include <ateam_geometry/types.hpp>
#include "base_play.hpp"
#include "play_helpers/easy_move_to.hpp"

namespace ateam_kenobi::plays
{

class WaypointsPlay : public BasePlay
{
public:
  WaypointsPlay();

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override;

private:
  struct Pose {
    ateam_geometry::Point position;
    double heading;
  };

  struct Waypoint
  {
    std::vector<Pose> poses;
    long int duration_ms;
  };

  std::array<play_helpers::EasyMoveTo, 16> easy_move_tos_;
  std::vector<Waypoint> waypoints_;
  std::chrono::steady_clock::time_point next_transition_time_ =
    std::chrono::steady_clock::time_point::max();
  std::size_t waypoint_index_ = 0;

  void addWaypoint(const long int duration_ms, const std::vector<std::tuple<double,double,double>> & poses);

};

} // namespace ateam_kenobi::plays

#endif  // PLAYS__WAYPOINTS_PLAY_HPP_
