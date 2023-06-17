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

#include "ateam_geometry/segment.hpp"
#include "ateam_geometry/utilities.hpp"
#include <Eigen/Dense>
#include <math.h>

namespace ateam_geometry {
    LineSegment::LineSegment(const Eigen::Vector2d & start, const Eigen::Vector2d & end) {
        p1 = start;
        p2 = end;

        // Length is Euclidian, we can change later if desired
        double length = std::sqrt(std::pow(p1.x() - p2.x(), 2) + std::pow(p1.y() - p2.y(), 2));
    }
    
    Eigen::Vector2d LineSegment::get_midpoint(){
        Eigen::Vector2d midpoint;
        midpoint.x() = (p1.x() + p2.x()) / 2;
        midpoint.y() = (p1.y() + p2.y()) / 2;
        return midpoint;
    }

    LineSegment get_lineseg_of_length_from_point(const Eigen::Vector2d & start, const double & length, const double & angle) {
        Eigen::Vector2d endpoint;
        // Essentially we convert a given polar coordinate/vector 
        // (length and angle) to an x, y plane
        endpoint.x() =  start.x() + (length * cos(angle));
        endpoint.y() = start.y() + (length * sin(angle));
        
        LineSegment segment = LineSegment(start, endpoint);

        return segment;
    }

    bool is_point_on_segment(const Eigen::Vector2d & point, const LineSegment & segment) {
        // TODO: Implement this
        return false;
    }

    bool do_segments_intersect(const LineSegment & ls1, const LineSegment & ls2) {
        /* Given two 2d line segments, return a bool corresponding to whether or not
        they intersect.

        Based on the algorithm implementation provided here:
        https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
        
        There are two line segments, one running from p to p + r and another spanning
        q to q + s.

        Any point on this first line can be represented as p + tr (with scalar
        parameter t) and any point on the second can be represented as q + us
        (with scalar parameter u).

        The two lines intersect if we can find t and u such that:
        p + t r = q + u s

        We can solve for t:
        t = (q − p) × s / (r × s)

        And we can solve for u:
        u = (q − p) × r / (r × s)

        (See Stack Overflow for the algebra, I'm lazy.)

        Based on these definitions, there are 4 possible cases of 
        whether or not 2 line segments intersect:

        1.If r x s = 0 && (q - p) x r = 0
            - The two segments are colinear
        2. If r x s = 0 && (q - p) x r != 0
            - Segments are parallel and non-intersecting
        3. If r x s != 0 && 0 <= t <= 1 && 0 <= u <= 1
            - Segments meet at p + tr = q + us
        4. Else
            - Segments are not parallel and do not intersect
        */
        Eigen::Vector2d p = ls1.p1;
        Eigen::Vector2d q = ls2.p1;
        Eigen::Vector2d r, s;

        r.x() = ls1.p2.x() - ls1.p1.x();
        r.y() = ls1.p2.y() - ls1.p1.y();

        s.x() = ls2.p2.x() - ls2.p1.x();
        s.y() = ls2.p2.y() - ls2.p1.y();

        double rxs = cross_product_2d(r, s);
        /* Need to change this to be within epsilon? */
        if (rxs == 0) {
            if (cross_product_2d(q - p, r) == 0) {
                return true;
            } else {
                return false;
            }
        } 
        else {
            double t = cross_product_2d(q - p, s) / rxs;
            double u = cross_product_2d(q - p, r) / rxs;
            if (0 <= t <= 1 && 0 <= u <= 1) {
                return true;
            } else {
                return false;
            }
        }
    }

    Eigen::Vector2d get_segment_intersection(const LineSegment & ls1, const LineSegment & ls2);
}