// Each camera has a multiple hypothesis tracker for each object
// 16 robots each team
// 1 ball
#include "filters/multiple_hypothesis_tracker.hpp"

#include <array>
#include <vector>

class Camera {
public:

private:
  std::array<std::vector<MultipleHypothesisTracker>, 16> yellow_team;
  std::array<std::vector<MultipleHypothesisTracker>, 16> blue_team;
  std::vector<MultipleHypothesisTracker> ball;
};