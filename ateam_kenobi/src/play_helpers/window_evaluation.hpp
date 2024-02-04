#ifndef PLAY_HELPERS__WINDOW_EVALUATOR_HPP_
#define PLAY_HELPERS__WINDOW_EVALUATOR_HPP_

#include <optional>
#include <vector>
#include <ateam_geometry/types.hpp>
#include "types/robot.hpp"
#include "visualization/overlays.hpp"

namespace ateam_kenobi::play_helpers::window_evaluation
{

std::vector<ateam_geometry::Segment> getWindows(
  const ateam_geometry::Segment & target,
  const ateam_geometry::Point & source,
  const std::vector<Robot> & robots);

std::optional<ateam_geometry::Segment> getLargestWindow(
  const std::vector<ateam_geometry::Segment> & windows);

std::optional<ateam_geometry::Segment> projectRobotShadowOntoLine(
  const Robot & robot,
  const ateam_geometry::Point & source,
  const ateam_geometry::Line & line);

void removeSegmentFromWindows(
  const ateam_geometry::Segment & seg,
  std::vector<ateam_geometry::Segment> & windows);

void drawWindows(
  const std::vector<ateam_geometry::Segment> & windows,
  const ateam_geometry::Point & source, visualization::Overlays overlays_);

}  // namespace ateam_kenobi::play_helpers::window_evaluation

#endif  // PLAY_HELPERS__WINDOW_EVALUATOR_HPP_
