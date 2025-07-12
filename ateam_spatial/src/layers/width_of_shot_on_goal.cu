#include "ateam_spatial/layers/width_of_shot_on_goal.hpp"
#include "ateam_spatial/layers/line_of_sight.hpp"
#include "ateam_spatial/coordinate_conversions.hpp"

namespace ateam_spatial::layers
{

CUDA_HOSTDEV float WidthOfShotOnGoal(const float x, const float y, const Robot * their_robots, const FieldDimensions & field_dims, const SpatialSettings & settings) {
  constexpr int kNumSteps = 100;

  const auto real_x = x;
  const auto real_y = y;

  const auto goal_x = field_dims.field_length / 2.0f;
  const auto goal_y_start = -field_dims.goal_width / 2.0f;

  const auto step_size = field_dims.goal_width / kNumSteps;

  int result = 0;
  int counter = 0;

  for(auto step = 0; step < kNumSteps; ++step) {
    const auto goal_y = goal_y_start + (step * step_size);
    if(LineOfSight(real_x, real_y, goal_x, goal_y, their_robots, field_dims, settings)) {
      counter++;
    } else {
      result = max(result, counter);
      counter = 0;
    }
  }
  result = max(result, counter);

  return result * step_size;
}
  
} // namespace ateam_spatial::layers
