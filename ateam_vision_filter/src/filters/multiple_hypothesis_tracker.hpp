// Matches measurements to filters
// Creates tracks as needed
// Removes tracks as needed
// Returns best track

#include "filters/interacting_multiple_model.hpp"

#include <vector>

class MultipleHypothesisTracker {
public:

private:
  std::vector<InteractingMultipleModel> tracks; // TODO: Make this the right data type
};