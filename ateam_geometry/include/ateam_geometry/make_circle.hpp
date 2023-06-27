
#ifndef ATEAM_GEOMETRY__MAKE_CIRCLE_HPP_
#define ATEAM_GEOMETRY__MAKE_CIRCLE_HPP_

#include "types.hpp"

namespace ateam_geometry
{

/**
 * @brief Factory utility to work around CGAL wanting the squared radius in the circle constructor.
 * 
 * @param center Center point of the circle
 * @param radius Radius of the circle
 * @return Circle 
 */
Circle makeCircle(Point center, double radius)
{
  return ateam_geometry::Circle(center, radius*radius);
}

}

#endif  // ATEAM_GEOMETRY__MAKE_CIRCLE_HPP_