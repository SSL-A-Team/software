// Each camera has a multiple hypothesis tracker for each object
// 16 robots each team
// 1 ball
#include "filters/multiple_hypothesis_tracker.hpp"
#include "types/ball_measurement.hpp"
#include "types/robot_measurement.hpp"

#include <array>
#include <vector>

class Camera {
public:

  void predict();
  void update(std::array<std::vector<RobotMeasurement>, 16> yellow_team_measurements,
              std::array<std::vector<RobotMeasurement>, 16> blue_team_measurements,
              std::vector<BallMeasurement> ball_measurements);

  // Some function to pass up best estimate and corresponding score

private:
  std::array<MultipleHypothesisTracker, 16> yellow_team;
  std::array<MultipleHypothesisTracker, 16> blue_team;
  MultipleHypothesisTracker ball;
};