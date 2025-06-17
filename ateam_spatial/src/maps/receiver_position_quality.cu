#include "ateam_spatial/maps/receiver_position_quality.hpp"
#include "ateam_spatial/layers/distance_down_field.hpp"
#include "ateam_spatial/layers/distance_from_field_edge.hpp"
#include "ateam_spatial/layers/distance_to_their_bots.hpp"
#include "ateam_spatial/layers/in_field.hpp"
#include "ateam_spatial/layers/line_of_sight_ball.hpp"
#include "ateam_spatial/layers/outside_their_defense_area.hpp"
#include "ateam_spatial/layers/width_of_shot_on_goal.hpp"
#include "ateam_spatial/coordinate_conversions.hpp"

namespace ateam_spatial::maps
{

CUDA_HOSTDEV float ReceiverPositionQuality(const int spatial_x, const int spatial_y, const Ball & ball, const Robot * their_bots, const FieldDimensions &field_dims, const SpatialSettings &settings)
{
  const auto x = SpatialToRealX(spatial_x, field_dims, settings);
  const auto y = SpatialToRealY(spatial_y, field_dims, settings);
  const auto max_edge_dist = 0.75;
  const auto edge_dist_multiplier = min(layers::DistanceFromFieldEdge(x, y, field_dims, settings), max_edge_dist) / max_edge_dist;
  const auto shot_width_multiplier = layers::WidthOfShotOnGoal(x, y, their_bots, field_dims, settings) / field_dims.goal_width;
  const auto robot_distance_multiplier = min(layers::DistanceToTheirBots(x, y, their_bots, field_dims, settings), 1.0);
  return layers::InField(x, y, field_dims, settings)
         * layers::OutsideTheirDefenseArea(x, y, field_dims, settings)
         * layers::LineOfSightBall(x, y, ball, their_bots, field_dims, settings)
         * edge_dist_multiplier
         * shot_width_multiplier
         * robot_distance_multiplier
         * layers::DistanceDownField(x, field_dims, settings);
}

} // namespace ateam_spatial::maps
