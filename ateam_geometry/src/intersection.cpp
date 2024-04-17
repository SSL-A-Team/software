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

#include "ateam_geometry/intersection.hpp"
#include "ateam_geometry/comparisons.hpp"

namespace ateam_geometry
{

std::optional<std::variant<Point, std::pair<Point, Point>>> intersection(
  const Circle & circle,
  const Line & line)
{
  // Implementation based on https://mathworld.wolfram.com/Circle-LineIntersection.html

  /* Algorithm assumes circle is centered at origin and line is defined using two points, so we
   * need to grab two arbitrary points along the line and make them relative to the circle's
   * center.
   */
  const auto line_p1 = line.point(0) - circle.center();
  const auto line_p2 = line.point(1) - circle.center();

  const auto line_direction = line.direction();
  const auto d_x = line_direction.dx();
  const auto d_y = line_direction.dy();
  const auto d_r = std::hypot(d_x, d_y);
  const auto D = (line_p1.x() * line_p2.y()) - (line_p2.x() * line_p1.y());
  const auto r = std::sqrt(circle.squared_radius());

  const auto descriminant = (r * r * d_r * d_r) - (D * D);

  const auto sqrt_descriminant = std::sqrt(descriminant);
  const auto d_r_sqrd = d_r * d_r;

  auto sgn = [](const double x) {return x < 0.0 ? -1.0 : 1.0;};

  const auto x1 = ((D * d_y) - (sgn(d_y) * d_x * sqrt_descriminant)) / d_r_sqrd;
  const auto x2 = ((D * d_y) + (sgn(d_y) * d_x * sqrt_descriminant)) / d_r_sqrd;
  const auto y1 = ((-D * d_x) - (std::abs(d_y) * sqrt_descriminant)) / d_r_sqrd;
  const auto y2 = ((-D * d_x) + (std::abs(d_y) * sqrt_descriminant)) / d_r_sqrd;

  if (std::abs(descriminant) < 1e-8) {
    // Tangent line
    return circle.center() + Vector(x1, y1);
  } else if (descriminant > 0.0) {
    // Secant line
    return std::make_pair(circle.center() + Vector(x1, y1), circle.center() + Vector(x2, y2));
  } else {
    // No intersection
    return std::nullopt;
  }
}

std::optional<std::variant<Point, std::pair<Point, Point>>> intersection(
  const Arc & arc,
  const Line & line)
{
  const auto circle_intersection =
    ateam_geometry::intersection(Circle(arc.center(), arc.radius() * arc.radius()), line);
  if (!circle_intersection) {
    return std::nullopt;
  }
  auto point_on_arc = [&arc](const auto & p) {
      const auto dir_to_p = (p - arc.center()).direction();
      return dir_to_p == arc.start() || dir_to_p == arc.end() ||
             dir_to_p.counterclockwise_in_between(arc.start(), arc.end());
    };
  if (std::holds_alternative<Point>(*circle_intersection)) {
    const auto & intersection_point = std::get<Point>(*circle_intersection);
    if (point_on_arc(intersection_point)) {
      return intersection_point;
    } else {
      return std::nullopt;
    }
  } else if (std::holds_alternative<std::pair<Point, Point>>(*circle_intersection)) {
    const auto & intersection_points = std::get<std::pair<Point, Point>>(*circle_intersection);
    const auto p1_valid = point_on_arc(intersection_points.first);
    const auto p2_valid = point_on_arc(intersection_points.second);
    if (p1_valid && p2_valid) {
      return intersection_points;
    } else if (p1_valid) {
      return intersection_points.first;
    } else if (p2_valid) {
      return intersection_points.second;
    } else {
      return std::nullopt;
    }
  }
  return std::nullopt;
}

std::optional<std::variant<Point, std::pair<Point, Point>>> intersection(
  const Arc & arc,
  const Ray & ray)
{
  const auto arc_intersection = ateam_geometry::intersection(arc, ray.supporting_line());

  if (!arc_intersection) {
    return std::nullopt;
  }

  // Not using Ray::has_on() because it does not account for small floating point mismatch
  auto point_on_ray = [&ray](const auto & p) {
      if (nearEqual(p, ray.source())) {return true;}
      const auto direction = (p - ray.source()).direction();
      return nearEqual(direction, ray.direction());
    };

  if (std::holds_alternative<Point>(*arc_intersection)) {
    const auto & intersection_point = std::get<Point>(*arc_intersection);
    if (point_on_ray(intersection_point)) {
      return intersection_point;
    } else {
      return std::nullopt;
    }
  } else if (std::holds_alternative<std::pair<Point, Point>>(*arc_intersection)) {
    const auto & intersection_points = std::get<std::pair<Point, Point>>(*arc_intersection);
    const auto p1_valid = point_on_ray(intersection_points.first);
    const auto p2_valid = point_on_ray(intersection_points.second);
    if (p1_valid && p2_valid) {
      return intersection_points;
    } else if (p1_valid) {
      return intersection_points.first;
    } else if (p2_valid) {
      return intersection_points.second;
    } else {
      return std::nullopt;
    }
  }

  return std::nullopt;
}

std::optional<std::variant<Point, std::pair<Point, Point>>> intersection(
  const Arc & arc,
  const Segment & segment)
{
  return std::nullopt;
}

}  // namespace ateam_geometry
