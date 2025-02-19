#ifndef LAYERS__ROBOT_SHADOWS_HPP_
#define LAYERS__ROBOT_SHADOWS_HPP_

#include <ateam_geometry/types.hpp>
#include "types/robot.hpp"
#include "types/world.hpp"

namespace ateam_kenobi::spatial::layers
{

ateam_geometry::Ray GetRobotTangentRay(const Robot & robot, const ateam_geometry::Point & source, const CGAL::Orientation & orientation);

std::pair<ateam_geometry::Ray, ateam_geometry::Ray> GetRobotShadowRays(const Robot & robot, const ateam_geometry::Point & source);
std::pair<ateam_geometry::Ray, ateam_geometry::Ray> GetRobotShadowRays(const Robot & robot, const ateam_geometry::Segment & source);

std::vector<ateam_geometry::Point> GetRobotShadowPoly(const Robot & robot, const ateam_geometry::Point & source, const World & world);
std::vector<ateam_geometry::Point> GetRobotShadowPoly(const Robot & robot, const ateam_geometry::Segment & source, const World & world);
  
} // namespace ateam_kenobi::spatial::layers

#endif  // LAYERS__ROBOT_SHADOWS_HPP_
