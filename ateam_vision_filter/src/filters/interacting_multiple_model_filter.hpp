// Multiple kalman filters each with a different model

// Ball with rolling accel
// Ball with sliding accel
// Ball with static velocity
// Ball with bounce velocity in estimate direction
// Ball with slow kick robot facing direction
// Ball with medium kick robot facing direction
// Ball with fast kick robot facing direciton

// Robot with constant accel

#include "kalman_filter.hpp"

#include <Eigen/Core>

#include <vector>

class InteractingMultipleModelFilter {
public:
  explicit InteractingMultipleModelFilter(const std::vector<KalmanFilter>& models);

  void predict();
  void update(Eigen::VectorXd measurement);

  Eigen::VectorXd get_state_estimate() const;

private:
  std::vector<KalmanFilter> models;
};