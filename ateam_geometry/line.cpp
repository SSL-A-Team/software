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
    Line::Line(const Eigen::Vector2d & start, const Eigen::Vector2d & end) {
        p1 = start;
        p2 = end;

        // Length is Euclidian, we can change later if desired
        double length = std::sqrt(std::pow(p1.x() - p2.x(), 2) + std::pow(p1.y() - p2.y(), 2));
    }
    
    Eigen::Vector2d Line::get_midpoint(){
        Eigen::Vector2d midpoint;

        midpoint.x() = (p1.x() + p2.x()) / 2;
        midpoint.y() = (p1.y() + p2.y()) / 2;

        return midpoint;
    }

    Line get_line_of_length_from_point(const Eigen::Vector2d & start, const double & length, const double & angle) {
        Eigen::Vector2d endpoint;
        endpoint.x() =  start.x() + (length * cos(angle));
        endpoint.y() = start.y() + (length * sin(angle));
        
        Line line = Line(start, endpoint);

        return line;
    };
}