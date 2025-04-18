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

#ifndef CORE__SPATIAL__SPATIAL_INSPECTION_HPP_
#define CORE__SPATIAL__SPATIAL_INSPECTION_HPP_

#include <ateam_geometry/types.hpp>
#include "core/types/field.hpp"
#include "spatial_map.hpp"

namespace ateam_kenobi::spatial
{

ateam_geometry::Point GetMaxPosition(const SpatialMap & map, const Field & field);

ateam_geometry::Point GetMaxPosition(
  const SpatialMap & map, const Field & field,
  const ateam_geometry::Rectangle & roi);

double GetValueAtLocation(
  const SpatialMap & map, const ateam_geometry::Point & location,
  const Field & field);

}  // namespace ateam_kenobi::spatial

#endif  // CORE__SPATIAL__SPATIAL_INSPECTION_HPP_
