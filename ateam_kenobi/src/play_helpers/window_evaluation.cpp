#include "window_evaluation.hpp"
#include <algorithm>
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/ateam_geometry.hpp>

namespace ateam_kenobi::play_helpers::window_evaluation
{

std::vector<ateam_geometry::Segment> getWindows(
  const ateam_geometry::Segment & target,
  const ateam_geometry::Point & source,
  const std::vector<Robot> & robots)
{
  std::vector<ateam_geometry::Segment> windows = {target};

  for (const auto & robot : robots) {
    const auto shadow = projectRobotShadowOntoLine(robot, source, target.supporting_line());
    if (shadow) {
      removeSegmentFromWindows(*shadow, windows);
    }
  }

  return windows;
}


std::optional<ateam_geometry::Segment> getLargestWindow(
  const std::vector<ateam_geometry::Segment> & windows)
{
  if (windows.empty()) {
    return std::nullopt;
  }
  return std::ranges::max(
    windows, [](const auto & w1, const auto & w2) {
      return w1.squared_length() < w2.squared_length();
    });
}

std::optional<ateam_geometry::Segment> projectRobotShadowOntoLine(
  const Robot & robot,
  const ateam_geometry::Point & source,
  const ateam_geometry::Line & line)
{
  const auto robot_vec = robot.pos - source;
  const auto perp_vec = robot_vec.perpendicular(CGAL::CLOCKWISE);
  const auto p1 = robot.pos + (kRobotRadius * perp_vec);
  const auto p2 = robot.pos + (-kRobotRadius * perp_vec);
  const ateam_geometry::Ray ray1(p1, p1 - source);
  const ateam_geometry::Ray ray2(p2, p2 - source);
  const auto maybe_intersect1 = CGAL::intersection(ray1, line);
  const auto maybe_intersect2 = CGAL::intersection(ray2, line);
  if (!maybe_intersect1 || !maybe_intersect2) {
    return std::nullopt;
  }
  const auto intersect1 = boost::get<ateam_geometry::Point>(&*maybe_intersect1);
  const auto intersect2 = boost::get<ateam_geometry::Point>(&*maybe_intersect2);
  if (!intersect1 || !intersect2) {
    return std::nullopt;
  }
  return ateam_geometry::Segment(*intersect1, *intersect2);
}

void removeSegmentFromWindows(
  const ateam_geometry::Segment & seg,
  std::vector<ateam_geometry::Segment> & windows)
{
  for (auto window_iter = windows.begin(); window_iter != windows.end(); ++window_iter) {
    const auto window = *window_iter;
    const auto maybe_intersection = CGAL::intersection(window, seg);
    if (!maybe_intersection) {
      continue;
    }
    if (const ateam_geometry::Segment * intersection_seg =
      boost::get<ateam_geometry::Segment>(&*maybe_intersection))
    {
      const auto & intersect_p1 = intersection_seg->source();
      const auto & intersect_p2 = intersection_seg->target();
      const auto p1_vec = intersect_p1 - window.source();
      const auto p2_vec = intersect_p2 - window.source();
      ateam_geometry::Segment new_seg_1;
      ateam_geometry::Segment new_seg_2;
      if (ateam_geometry::norm(p1_vec) < ateam_geometry::norm(p2_vec)) {
        new_seg_1 = ateam_geometry::Segment(window.source(), intersect_p1);
        new_seg_2 = ateam_geometry::Segment(intersect_p2, window.target());
      } else {
        new_seg_1 = ateam_geometry::Segment(window.source(), intersect_p2);
        new_seg_2 = ateam_geometry::Segment(intersect_p1, window.target());
      }
      window_iter = windows.erase(window_iter);
      if (new_seg_2.squared_length() > 1e-4) {
        window_iter = windows.insert(window_iter, new_seg_2);
      }
      if (new_seg_1.squared_length() > 1e-4) {
        window_iter = windows.insert(window_iter, new_seg_1);
      }
    }
  }
}

void drawWindows(
  const std::vector<ateam_geometry::Segment> & windows,
  const ateam_geometry::Point & source, visualization::Overlays overlays_)
{
  auto i = 0;
  for (const auto & window : windows) {
    ateam_geometry::Polygon window_triangle;
    window_triangle.push_back(source);
    window_triangle.push_back(window.source());
    window_triangle.push_back(window.target());
    overlays_.drawPolygon("Window" + std::to_string(i), window_triangle, "00000000", "0000FF7F");
    ++i;
  }
}

}  // namespace ateam_kenobi::play_helpers::window_evaluation
