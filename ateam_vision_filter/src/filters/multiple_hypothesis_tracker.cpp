#include "filters/multiple_hypothesis_tracker.hpp"

void MultipleHypothesisTracker::set_base_track(const InteractingMultipleModelFilter & base_track)
{
  this->base_track = base_track;
}

void MultipleHypothesisTracker::predict()
{
  for (auto& track : tracks) {
    track.predict();
  }
}

void MultipleHypothesisTracker::update(const std::vector<Eigen::VectorXd> & measurements)
{
  // Data association
  // minimize assignment cost where cost is different between each
  // model's predicted location and the measurement location
  //
  // Form is a bipartite graph assignmnet problem with 1 supply per measurement,
  // and 1 sink per track
}

void MultipleHypothesisTracker::life_cycle_management()
{
  // Life cycle management
  // Anything that's missed too many should be removed
}