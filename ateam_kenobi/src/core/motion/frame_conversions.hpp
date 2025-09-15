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


#ifndef CORE__MOTION__FRAME_CONVERSIONS_HPP_
#define CORE__MOTION__FRAME_CONVERSIONS_HPP_

#include <ateam_geometry/types.hpp>
#include "core/types/robot.hpp"

namespace ateam_kenobi::motion
{

ateam_geometry::Vector WorldToLocalFrame(
  const ateam_geometry::Vector & world_vector,
  const Robot & robot);

ateam_geometry::Vector LocalToWorldFrame(
  const ateam_geometry::Vector & local_vector,
  const Robot & robot);

}  // namespace ateam_kenobi::motion

#endif  // CORE__MOTION__FRAME_CONVERSIONS_HPP_
