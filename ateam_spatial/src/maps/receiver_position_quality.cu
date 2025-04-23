#include "ateam_spatial/maps/receiver_position_quality.hpp"
#include "ateam_spatial/layers/distance_down_field.hpp"
#include "ateam_spatial/layers/distance_from_field_edge.hpp"
#include "ateam_spatial/layers/in_field.hpp"

namespace ateam_spatial::maps
{

CUDA_HOSTDEV float ReceiverPositionQuality(const int x, const int y, const FieldDimensions &field_dims, const SpatialSettings &settings)
{
  const auto max_edge_dist = 0.75;
  const auto edge_dist_multiplier = min(layers::DistanceFromFieldEdge(x, y, field_dims, settings), max_edge_dist) / max_edge_dist;
  return layers::InField(x, y, field_dims, settings) * edge_dist_multiplier * layers::DistanceDownField(x, field_dims, settings);
}

} // namespace ateam_spatial::maps
