#include "filters/multiple_hypothesis_tracker.hpp"
#include "types/camera_measurement.hpp"

#include <array>
#include <optional>
#include <vector>

class Camera {
public:
  using BallWithScore = std::pair<Ball, double>;
  using RobotWithScore = std::pair<Robot, double>;

  Camera();

  /**
   * Updates the camera with a specific frame's measurement
   * 
   * @param camera_measurement Measurement from a specific frame
   */
  void update(const CameraMeasurement & camera_measurement);

  /**
   * Step froward the camera physics models one time step
   */
  void predict();

  /**
   * @return Returns a ball with a corresponding likelyhood score (if exists)
   */
  std::optional<BallWithScore> get_ball_estimate_with_score();
  
  /**
   * @return Returns yellow robots with a corresponding likelyhood score (if exists)
   */
  std::array<std::optional<RobotWithScore>, 16> get_yellow_robot_estimates_with_score();

  /**
   * @return Returns blue robots with a corresponding likelyhood score (if exists)
   */
  std::array<std::optional<RobotWithScore>, 16> get_blue_robot_estimates_with_score();

private:
  static std::vector<Eigen::VectorXd> robot_measurements_to_vector(
    const std::vector<RobotMeasurement> & robot_measurements);

  static std::vector<Eigen::VectorXd> ball_measurements_to_vector(
    const std::vector<BallMeasurement> & ball_measurements);

  std::array<MultipleHypothesisTracker, 16> yellow_team;
  std::array<MultipleHypothesisTracker, 16> blue_team;
  MultipleHypothesisTracker ball;
};