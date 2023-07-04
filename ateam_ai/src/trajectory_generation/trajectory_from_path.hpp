#ifndef TRAJECTORY_GENERATION__TRAJECTORY_FROM_PATH__
#define TRAJECTORY_GENERATION__TRAJECTORY_FROM_PATH__

#include <Eigen/Dense>
#include "types/trajectory.hpp"

namespace ateam_ai::trajectory_generation
{

Trajectory TrajectoryFromPath(const std::vector<Eigen::Vector2d> & path, const double current_time, const Eigen::Vector3d & end_vel, const double max_vel)
{
  Trajectory trajectory;

  if(path.empty()) {
    return trajectory;
  }

  auto sample_time = current_time;

  for(auto path_ind = 0u; path_ind < path.size()-1; path_ind++) {
    Sample3d sample;

    sample.pose.head(2) = path[path_ind];

    const Eigen::Vector2d segment_vector = path[path_ind + 1] - path[path_ind];

    sample.vel.head(2) = segment_vector.normalized() * max_vel;

    sample.time = sample_time;
    sample_time += segment_vector.norm() / max_vel;

    trajectory.samples.push_back(sample);
  }

  Sample3d last_sample;
  last_sample.pose.head(2) = path.back();
  last_sample.vel = end_vel;
  last_sample.time = sample_time;
  trajectory.samples.push_back(last_sample);

  return trajectory;
}

}  // namespace ateam_ai::trajectory_generation

#endif  // TRAJECTORY_GENERATION__TRAJECTORY_FROM_PATH__
