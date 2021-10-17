// Matches measurements to filters
// Creates tracks as needed
// Removes tracks as needed
// Returns best track

#include "filters/interacting_multiple_model_filter.hpp"

#include <Eigen/Dense>

#include <vector>

class MultipleHypothesisTracker {
public:
  void set_base_track(const InteractingMultipleModelFilter & base_track);

  void update(const std::vector<Eigen::VectorXd> & measurements);
  void predict();

private:
  void life_cycle_management();

  InteractingMultipleModelFilter base_track;

  std::vector<InteractingMultipleModelFilter> tracks;
};
