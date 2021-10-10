// Matches measurements to filters
// Creates tracks as needed
// Removes tracks as needed
// Returns best track

#include "filters/interacting_multiple_model_filter.hpp"

#include <vector>

template <class Measurement>
class MultipleHypothesisTracker {
public:

  void predict();
  void update(std::vector<Measurement> measurements);

private:
  void life_cycle_management();

  std::vector<InteractingMultipleModelFilter> tracks; // TODO: Make this the right data type
};

template <class Measurement>
void MultipleHypothesisTracker::predict()
{
  for (auto& track : tracks) {
    track.predict();
  }
}

template <class Measurement>
void MultipleHypothesisTracker::update(std::vector<Measurement> measurements)
{
  // Data association
  // minimize assignment cost where cost is different between each
  // model's predicted location and the measurement location
  //
  // Form is a bipartite graph assignmnet problem with 1 supply per measurement,
  // and 1 sink per track
}

template <class Measurement>
void MultipleHypothesisTracker::life_cycle_management()
{
  // Life cycle management
  // Anything that's missed too many should be removed
}