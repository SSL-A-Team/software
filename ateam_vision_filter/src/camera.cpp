#include "camera.hpp"

Camera::Camera()
{

}

void Camera::update(const CameraMeasurement & camera_measurement)
{
    for (size_t robot_id = 0; robot_id < 16; robot_id++)
    {
        if (!camera_measurement.yellow_robots.at(robot_id).empty())
        {
            std::vector<Eigen::VectorXd> vectored_measurements = 
                robot_measurements_to_vector(camera_measurement.yellow_robots.at(robot_id));
            yellow_team.at(robot_id).update(vectored_measurements);
        }

        if (!camera_measurement.blue_robots.at(robot_id).empty())
        {
            std::vector<Eigen::VectorXd> vectored_measurements = 
                robot_measurements_to_vector(camera_measurement.blue_robots.at(robot_id));
            blue_team.at(robot_id).update(vectored_measurements);
        }
    }

    if (!camera_measurement.ball.empty())
    {
        std::vector<Eigen::VectorXd> vectored_measurements =
            ball_measurements_to_vector(camera_measurement.ball);
        ball.update(vectored_measurements);
    }
}

void Camera::predict()
{
    for (auto & yellow_robot : yellow_team)
    {
        yellow_robot.predict();
    }

    for (auto & blue_robot : blue_team)
    {
        blue_robot.predict();
    }

    ball.predict();
}

std::optional<Camera::BallWithScore> Camera::get_ball_estimate_with_score()
{
    // TODO: Return from the tracker
    return std::nullopt;
}

std::array<std::optional<Camera::RobotWithScore>, 16> Camera::get_yellow_robot_estimates_with_score()
{
    // TODO: Return from the tracker
}

std::array<std::optional<Camera::RobotWithScore>, 16> Camera::get_blue_robot_estimates_with_score()
{
    // TODO: Return from the tracker
}

std::vector<Eigen::VectorXd> Camera::robot_measurements_to_vector(
    const std::vector<RobotMeasurement> & robot_measurements)
{
    std::vector<Eigen::VectorXd> vectored_measurements;
    vectored_measurements.reserve(robot_measurements.size());

    std::transform(robot_measurements.begin(), robot_measurements.end(),
        vectored_measurements.begin(),
        [] (const RobotMeasurement & original) {
            return Eigen::Vector3d{original.position.x(), original.position.y(), original.theta};
        });

    return vectored_measurements;
}

std::vector<Eigen::VectorXd> Camera::ball_measurements_to_vector(
    const std::vector<BallMeasurement> & ball_measurements)
{
    std::vector<Eigen::VectorXd> vectored_measurements;
    vectored_measurements.reserve(ball_measurements.size());

    std::transform(ball_measurements.begin(), ball_measurements.end(),
        vectored_measurements.begin(),
        [] (const BallMeasurement & original) {
            return original.position;
        });

    return vectored_measurements;
}