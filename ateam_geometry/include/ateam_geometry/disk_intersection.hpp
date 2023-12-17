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

/**
 * This module exists because the geometry library we use (CGAL) doesn't
 * treat circles consistently as open or closed. Here we have intersection
 * code that consistently treats them as disks (closed circles).
 * See Wikipedia if you want to read more:
 * https://en.wikipedia.org/wiki/Disk_(mathematics)
*/

#ifndef ATEAM_GEOMETRY__DISK_INTERSECTION_HPP_
#define ATEAM_GEOMETRY__DISK_INTERSECTION_HPP_

#include <CGAL/Distance_2/Point_2_Segment_2.h>
#include <variant>
#include "ateam_geometry/types.hpp"

namespace ateam_geometry
{

/**
 * @brief Disk intersection that accounts for the degenerate case where
 * one disk is completely inside the other.
 *
 * @param disk1 A circle including its interior (disk)
 * @param disk2 A different circle including its interior (disk)
 */
inline bool diskDiskIntersection(const Circle & disk1,
                                 const Circle & disk2)
{
  double sr1 = disk1.squared_radius();
  double sr2 = disk2.squared_radius();
  double squared_dist = squared_distance(disk1.center(), disk2.center());

  // Check if one circle is completely inside the other
  if (squared_dist <= std::max(sr1, sr2)) {
    return true;
  }

  double temp = sr1 + sr2 - squared_dist;

  return !(4 * sr1 * sr2 < temp * temp);
}

/**
 * @brief Disk-rectangle intersection that accounts for the degenerate case where
 * the disk is completely inside the rectangle.
 *
 * @param disk A circle including its interior (disk)
 * @param rec A rectangle, including its interior
 */
inline bool diskRectangleIntersection(const Circle & disk,
                                      const Rectangle & rec)
{
  Point center = disk.center();

  // First, to address degenerate cases:
  // Determine if the center of the circle is within the rectangle
  if (center.x() >= rec.xmin() && center.x() <= rec.xmax() &&
    center.y() >= rec.ymin() && center.y() <= rec.ymax()) {
    return true;
  }

  // Then, deal with rectangle in circle...
  // Check that the minimum distance to the box is smaller than the radius, otherwise there is
  // no intersection.
  double sq_distance = 0;
  if(center.x() < rec.xmin())
  {
    double d = rec.xmin() - center.x();
    sq_distance += d * d;
  }
  else if(center.x() > rec.xmax())
  {
    double d = center.x() - rec.xmax();
    sq_distance += d * d;
  }

  if(center.y() < rec.ymin())
  {
    double d = rec.ymin() - center.y();
    sq_distance += d * d;
  }
  else if(center.y() > rec.ymax())
  {
    double d = center.y() - rec.ymax();
    sq_distance += d * d;
  }

  // Note that with the way the distance above is computed, the distance is '0' if the box strictly
  // contains the circle. But since we use '>', we don't exit
  if(sq_distance > disk.squared_radius()){
    return false;
  }

  // Check that the maximum distance between the center of the circle and the box is not (strictly)
  // smaller than the radius of the center, otherwise the box is entirely contained.
  double sq_distance = 0;
  if(center.x() <= (rec.xmin() + rec.xmax()) / 2)
  {
    double d = rec.xmax() - center.x();
    sq_distance += d * d;
  }
  else
  {
    double d = center.x() - rec.xmin();
    sq_distance += d * d;
  }

  if(center.y() < (rec.ymin() + rec.ymax()) / 2)
  {
    double d = rec.ymax() - center.y();
    sq_distance += d * d;
  }
  else
  {
    double d = center.y() - rec.ymin();
    sq_distance += d * d;
  }
  return (sq_distance >= disk.squared_radius());
}

/**
 * @brief Disk-point intersection that assumes a circle is closed.
 *
 * @param disk A circle including its interior (disk)
 * @param point A point in 2D space
 */
inline bool diskPointIntersection(const Circle & disk,
                                  const Point & point){
    Point center = disk.center();

    double sq_distance = 0;

    if (point.x() < center.x()){
        double d = center.x() - point.x();
        sq_distance += d * d;
    } else if (point.x() > center.x())
    {
        double d = point.x() - center.x();
        sq_distance += d * d;
    }

    if (point.y() < center.y()){
        double d = center.y() - point.y();
        sq_distance += d * d;
    } else if (point.y() > center.y())
    {
        double d = point.y() - center.y();
        sq_distance += d * d;
    }

    return (sq_distance <= disk.squared_radius());
}

/**
 * @brief Disk-line segment intersection. The CGAL case assumes the interior
 * of a circle is a disk, so we can just reuse it.
 *
 * @param disk A circle including its interior (disk)
 * @param segment A 2D line segment
 */
inline bool segmentDiskIntersection(const Circle & disk,
                                    const Segment & segment) {
    return CGAL::do_intersect(disk, segment);
}

/**
 * @brief Disk-ray intersection. The CGAL case assumes the interior
 * of a circle is a disk, so we can just reuse it.
 *
 * @param disk A circle including its interior (disk)
 * @param segment A 2D line segment
 */
inline bool rayDiskIntersection(const Circle & disk,
                                const Ray & ray){
    return CGAL::do_intersect(disk, ray);
}

}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__DISK_INTERSECTION_HPP_