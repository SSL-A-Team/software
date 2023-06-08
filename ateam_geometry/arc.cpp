// Copyright 2023 A Team
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

#include "ateam_geometry/line.hpp"
#include <Eigen/Dense>
#include <math.h>

namespace ateam_geometry {
    Arc::Arc(const Eigen::Vector2d p1, const Eigen::Vector2d p2, const double radius){
        Eigen::Vector2d p1 = p1;
        Eigen::Vector2d p2 = p2;
        float radius = radius;

        // Using law of cosines to find the angle of the arc
        Line a = Line(p1, p2);
        float angle = acos((2 * pow(radius, 2) - pow(a.length, 2)) / (2 * pow(radius, 2)));

        Line r = get_line_of_length_from_point(p1, radius, );

    }


    std::vector<Eigen::Vector2d> Arc::get_equally_spaced_points(int num_points){
        // Divide the angle into the given number of equally sized angles
        double angle_size = self.angle / num_points;

        // Start from the first endpoint
        


    };
}