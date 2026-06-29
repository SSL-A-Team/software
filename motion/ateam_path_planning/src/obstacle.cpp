// Copyright 2026 A Team
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

#include "ateam_path_planning/obstacle.hpp"

namespace ateam_path_planning
{

template<typename T>
T TranslateShape(const T & in, const ateam_geometry::Vector & displacement)
{
  CGAL::Aff_transformation_2<ateam_geometry::Kernel> translation{CGAL::TRANSLATION, displacement};
  return translation(in);
}

template<>
ateam_geometry::Circle TranslateShape<ateam_geometry::Circle>(
  const ateam_geometry::Circle & in,
  const ateam_geometry::Vector & displacement)
{
  return ateam_geometry::Circle{in.center() + displacement, in.squared_radius()};
}

template<>
ateam_geometry::Disk TranslateShape<ateam_geometry::Disk>(
  const ateam_geometry::Disk & in,
  const ateam_geometry::Vector & displacement)
{
  return ateam_geometry::Disk{in.center() + displacement, in.squared_radius()};
}

template<>
ateam_geometry::Ray TranslateShape<ateam_geometry::Ray>(
  const ateam_geometry::Ray & in,
  const ateam_geometry::Vector & displacement)
{
  return ateam_geometry::Ray{in.source() + displacement, in.direction()};
}

template<>
ateam_geometry::Rectangle TranslateShape<ateam_geometry::Rectangle>(
  const ateam_geometry::Rectangle & in, const ateam_geometry::Vector & displacement)
{
  return ateam_geometry::Rectangle{in.min() + displacement, in.max() + displacement};
}

template<>
ateam_geometry::Segment TranslateShape<ateam_geometry::Segment>(
  const ateam_geometry::Segment & in,
  const ateam_geometry::Vector & displacement)
{
  return ateam_geometry::Segment{in.start() + displacement, in.end() + displacement};
}

ateam_geometry::AnyShape Obstacle::ShapeAtT(const double t) const
{
  if(std::holds_alternative<std::monostate>(expected_motion)) {
    return shape;
  }

  ateam_geometry::Vector displacement;

  if(const auto velocity = std::get_if<ateam_geometry::Vector>(&expected_motion);
    velocity != nullptr)
  {
    displacement = (*velocity) * t;
  }

  if(const auto trajectory = std::get_if<ObstacleTrajectory>(&expected_motion);
    trajectory != nullptr)
  {
    const auto index = static_cast<size_t>(t / trajectory->time_step);
    if (index >= trajectory->points.size() - 1) {
      displacement = trajectory->points.back() - trajectory->points.front();
    } else {
      const auto remainder = (t / trajectory->time_step) - index;
      const auto interpolation_offset = (trajectory->points[index + 1] -
        trajectory->points[index]) * remainder;
      const auto interpolated_point = trajectory->points[index] + interpolation_offset;
      displacement = interpolated_point - trajectory->points.front();
    }
  }

  return std::visit([&displacement](const auto & s){
             return ateam_geometry::AnyShape{TranslateShape(s, displacement)};
    }, shape);
}

}  // namespace ateam_path_planning
