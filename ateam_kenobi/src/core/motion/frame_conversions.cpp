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


#include "frame_conversions.hpp"
#include <CGAL/Aff_transformation_2.h>

namespace ateam_kenobi::motion
{

ateam_geometry::Vector WorldToLocalFrame(
  const ateam_geometry::Vector & world_vector,
  const Robot & robot)
{
  CGAL::Aff_transformation_2<ateam_geometry::Kernel> transformation(CGAL::ROTATION,
    std::sin(-robot.theta), std::cos(-robot.theta));
  return world_vector.transform(transformation);
}

ateam_geometry::Vector LocalToWorldFrame(
  const ateam_geometry::Vector & local_vector,
  const Robot & robot)
{
  CGAL::Aff_transformation_2<ateam_geometry::Kernel> transformation(CGAL::ROTATION,
    std::sin(robot.theta), std::cos(robot.theta));
  return local_vector.transform(transformation);
}

}  // namespace ateam_kenobi::motion
