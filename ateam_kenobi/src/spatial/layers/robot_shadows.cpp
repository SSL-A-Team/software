#include "robot_shadows.hpp"
#include <angles/angles.h>
#include <ateam_common/equality_utilities.hpp>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/ateam_geometry.hpp>

namespace ateam_kenobi::spatial::layers
{

std::vector<ateam_geometry::Point> GetWorldCorners(const World & world)
{
  const auto half_world_length = (world.field.field_length / 2.0) + world.field.boundary_width;
  const auto half_world_width = (world.field.field_width / 2.0) + world.field.boundary_width;
  return {
    ateam_geometry::Point{half_world_length, half_world_width},
    ateam_geometry::Point{-half_world_length, half_world_width},
    ateam_geometry::Point{-half_world_length, -half_world_width},
    ateam_geometry::Point{half_world_length, -half_world_width},
  };
}

double AngleOfPoint(const ateam_geometry::Point & p)
{
  return angles::normalize_angle_positive(std::atan2(p.y(), p.x()));
}

// Note: All shadows are assumed to span clockwise
void AddShadowedFieldCorners(
  std::vector<ateam_geometry::Point> & points,
  const ateam_geometry::Point & shadow_start, const ateam_geometry::Point & shadow_end,
  const World & world)
{
  const auto start_angle = AngleOfPoint(shadow_start);
  const auto end_angle = AngleOfPoint(shadow_end);
  const auto world_corners = GetWorldCorners(world);
  std::vector<double> world_corner_angles;
  std::ranges::transform(world_corners, std::back_inserter(world_corner_angles), AngleOfPoint);
  for(auto search_start_index = 0ul; search_start_index < 4; ++search_start_index) {
    if(ateam_geometry::IsClockwiseBetween(start_angle, world_corner_angles[search_start_index],
        world_corner_angles[(search_start_index + 1) % 4]))
    {
      for(auto search_index = 0ul; search_index < 4; ++search_index) {
        const auto corner_index = (search_index + search_start_index) % 4;
        const auto & corner_angle = world_corner_angles[corner_index];
        if(ateam_geometry::IsClockwiseBetween(corner_angle, start_angle, end_angle)) {
          points.push_back(world_corners[corner_index]);
        }
      }
      break;
    }
  }
}

std::vector<ateam_geometry::Point> GetNonconvergingShadowPoly(
  const std::pair<ateam_geometry::Ray,
  ateam_geometry::Ray> & rays, const World & world)
{
  const auto & [ray_1, ray_2] = rays;
  const auto half_world_length = (world.field.field_length / 2.0) + world.field.boundary_width;
  const auto half_world_width = (world.field.field_width / 2.0) + world.field.boundary_width;
  const ateam_geometry::Rectangle world_bounds{
    ateam_geometry::Point{-half_world_length, -half_world_width},
    ateam_geometry::Point{half_world_length, half_world_width}
  };
  const auto intersection_1 = ateam_geometry::intersection(ray_1, world_bounds);
  const auto intersection_2 = ateam_geometry::intersection(ray_2, world_bounds);
  if(!intersection_1 || !intersection_2) {
    return {};
  }
  const auto intersection_1_segment = boost::get<ateam_geometry::Segment>(&*intersection_1);
  const auto intersection_2_segment = boost::get<ateam_geometry::Segment>(&*intersection_2);
  if(!intersection_1_segment || !intersection_2_segment) {
    return {};
  }
  const auto intersection_1_point = intersection_1_segment->target();
  const auto intersection_2_point = intersection_2_segment->target();
  std::vector<ateam_geometry::Point> points{
    ray_1.source(),
    intersection_1_point
  };
  AddShadowedFieldCorners(points, intersection_1_point, intersection_2_point, world);
  points.push_back(intersection_2_point);
  points.push_back(ray_2.source());
  return points;
}

ateam_geometry::Ray GetRobotTangentRay(
  const Robot & robot, const ateam_geometry::Point & source,
  const CGAL::Orientation & orientation)
{
  const auto source_center_vector = robot.pos - source;
  const auto source_center_distance = ateam_geometry::norm(source_center_vector);
  const auto shadow_angle = std::asin(kRobotRadius / source_center_distance);
  CGAL::Aff_transformation_2<ateam_geometry::Kernel> rotate(CGAL::ROTATION, std::sin(shadow_angle),
    std::cos(shadow_angle));
  if(orientation == CGAL::COUNTERCLOCKWISE) {
    rotate = rotate.inverse();
  }
  const auto source_to_bot_tangent_distance = std::hypot(source_center_distance, kRobotRadius);
  const auto shadow_vector = ateam_geometry::normalize(rotate(source_center_vector));
  const auto tangent_point = source + (shadow_vector * source_to_bot_tangent_distance);
  return ateam_geometry::Ray(tangent_point, shadow_vector);
}

std::pair<ateam_geometry::Ray, ateam_geometry::Ray> GetRobotShadowRays(
  const Robot & robot,
  const ateam_geometry::Point & source)
{
  // Orientation order matters to consistently yield clockwise-oriented ray pairs
  return std::make_pair(
    GetRobotTangentRay(robot, source, CGAL::COUNTERCLOCKWISE),
    GetRobotTangentRay(robot, source, CGAL::CLOCKWISE)
  );
}

std::pair<ateam_geometry::Ray, ateam_geometry::Ray> GetRobotShadowRays(
  const Robot & robot,
  const ateam_geometry::Segment & source)
{
  // TODO(barulicm) dynamically figure out which point should be used for each orientation
  return std::make_pair(
    GetRobotTangentRay(robot, source.target(), CGAL::COUNTERCLOCKWISE),
    GetRobotTangentRay(robot, source.source(), CGAL::CLOCKWISE)
  );
}

std::vector<ateam_geometry::Point> GetRobotShadowPoly(
  const Robot & robot,
  const ateam_geometry::Point & source, const World & world)
{
  return GetNonconvergingShadowPoly(GetRobotShadowRays(robot, source), world);
}

std::vector<ateam_geometry::Point> GetRobotShadowPoly(
  const Robot & robot,
  const ateam_geometry::Segment & source, const World & world)
{
  if(source.squared_length() < kRobotDiameter * kRobotDiameter) {
    // Diverging or parallel shadow
    return GetNonconvergingShadowPoly(GetRobotShadowRays(robot, source), world);
  } else {
    // Converging shadow
    const auto [top_shadow_ray, bottom_shadow_ray] = GetRobotShadowRays(robot, source);
    const auto maybe_intersection = ateam_geometry::intersection(top_shadow_ray, bottom_shadow_ray);
    if(!maybe_intersection) {
      return {};
    }
    const auto intersection_point = boost::get<ateam_geometry::Point>(&*maybe_intersection);
    if(!intersection_point) {
      return {};
    }
    return std::vector<ateam_geometry::Point>{
      top_shadow_ray.source(),
      bottom_shadow_ray.source(),
      *intersection_point
    };
  }
}

} // namespace ateam_kenobi::spatial::layers
