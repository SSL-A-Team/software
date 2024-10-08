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

#ifndef ATEAM_GEOMETRY__EPSILON_HPP_
#define ATEAM_GEOMETRY__EPSILON_HPP_

namespace ateam_geometry
{

/// @brief Epsilon for treating a generic float as effectively 0
const double kGenericEpsilon = 1e-8;

/// @brief Epsilon under which an angle is effectively 0. In radians.
/// Set to 0.01 degrees
const double kAngleEpsilon = 1.75e-4;

/// @brief Epsilon under which a distance is effectively 0. In meters.
/// Set to 1 micrometer
const double kDistanceEpsilon = 1e-6;

}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__EPSILON_HPP_
