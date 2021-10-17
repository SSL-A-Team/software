// Contains all the cameras

#include "camera.hpp"
#include "types/ball.hpp"
#include "types/camera_measurement.hpp"
#include "types/robot.hpp"

#include <array>
#include <map>
#include <optional>

class World {
public:
  using CameraID = int;

  /**
   * Updates the world with a specific camera's measurement
   * 
   * @param cameraID Unique ID of the camera given by SSL Vision
   * @param measurement Measurement from the camera frame
   */
  void update_camera(const CameraID & cameraID, const CameraMeasurement & measurement);

  /**
   * Step forward the world physics models one time step
   */
  void predict();

  /**
   * @return The best possible estimate for the ball (if one exists)
   */
  std::optional<Ball> get_ball_estimate();

  /**
   * @return The best possible estimate for each yellow robot (if one exists)
   */
  std::array<std::optional<Robot>, 16> get_yellow_robots_estimate();

  /**
   * @return The best possible estimate for each blue robot (if one exists)
   */
  std::array<std::optional<Robot>, 16> get_blue_robots_estimate();

private:
  std::map<CameraID, Camera> cameras;
};