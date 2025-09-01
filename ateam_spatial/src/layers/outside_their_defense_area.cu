#include "ateam_spatial/layers/outside_their_defense_area.hpp"
#include "ateam_spatial/coordinate_conversions.hpp"

namespace ateam_spatial::layers
{

CUDA_HOSTDEV float OutsideTheirDefenseArea(const float x, const float y, const FieldDimensions & field_dims, const SpatialSettings & settings) {
  const auto robot_diameter = 0.180f;
  const auto pos_x_thresh = field_dims.field_length / 2.0f;
  const auto neg_x_thresh = (field_dims.field_length / 2.0f) - (field_dims.defense_area_depth + (robot_diameter * 3.0f));
  const auto pos_y_thresh = (field_dims.defense_area_width / 2.0f) + (robot_diameter * 3.0f);
  const auto neg_y_thresh = -pos_y_thresh;

  return !((x <= pos_x_thresh) && (x >= neg_x_thresh) && (y <= pos_y_thresh) && (y >= neg_y_thresh));
}
  
} // namespace ateam_spatial::layers
