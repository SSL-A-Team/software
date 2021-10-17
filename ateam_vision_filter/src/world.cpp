#include "world.hpp"

#include <utility>

void World::update_camera(const CameraID & cameraID, const CameraMeasurement & measurement)
{
    // If camera does not exists, add it
    if (cameras.find(cameraID) == cameras.end())
    {
        cameras[cameraID] = Camera();
    }

    // Update the specific camera
    cameras[cameraID].update(measurement);
}

void World::predict()
{
    for (auto & camera_pair : cameras)
    {
        camera_pair.second.predict();
    }
}

std::optional<Ball> World::get_ball_estimate()
{
    using BallWithScore = std::pair<Ball, double>;
    std::vector<BallWithScore> balls_with_scores;

    // Grab estimates from cameras
    for (auto & camera_pair : cameras)
    {
        std::optional<BallWithScore> possible_ball_with_score
            = camera_pair.second.get_ball_estimate_with_score();
        
        if (possible_ball_with_score.has_value())
        {
            balls_with_scores.emplace_back(possible_ball_with_score.value());
        }
    }

    // If we have no estimates, there is no ball
    if (balls_with_scores.empty())
    {
        return std::nullopt;
    }

    // TODO: Merge balls based on score
    Ball merged_ball;

    return merged_ball;
}

std::array<std::optional<Robot>, 16> World::get_yellow_robots_estimate()
{
    using RobotWithScore = std::pair<Robot, double>;
    std::array<std::vector<RobotWithScore>, 16> yellow_robots_with_scores;

    // Grab estimates from camera
    for (auto & camera_pair : cameras)
    {
        std::array<std::optional<RobotWithScore>, 16> possible_yellow_robots_with_score
            = camera_pair.second.get_yellow_robot_estimates_with_score();

        for (size_t yellow_id = 0; yellow_id < 16; yellow_id++)
        {
            if (possible_yellow_robots_with_score.at(yellow_id).has_value())
            {
                yellow_robots_with_scores.at(yellow_id).emplace_back(
                    possible_yellow_robots_with_score.at(yellow_id).value());
            }
        }
    }

    // Try to merge them together
    // return nullopt if there are no measurements
    std::array<std::optional<Robot>, 16> yellow_robots_estimates;
    for (size_t yellow_id = 0; yellow_id < 16; yellow_id++)
    {
        if (yellow_robots_with_scores.empty())
        {
            yellow_robots_estimates.at(yellow_id) = std::nullopt;
        }
        else
        {
            // Merge robots based on weighted average of their scores
            yellow_robots_estimates.at(yellow_id) = Robot();

            auto & output_robot = yellow_robots_estimates.at(yellow_id).value();
            double total_score = 0.0;

            for (const auto & robot_with_score : yellow_robots_with_scores.at(yellow_id))
            {
                const Robot & robot = std::get<0>(robot_with_score);
                const double & score = std::get<1>(robot_with_score);
                output_robot.position += score * robot.position;
                // TODO Angle average
                output_robot.velocity += score * robot.velocity;
                output_robot.omega += score * robot.omega;
                output_robot.acceleration += score * robot.acceleration;
                output_robot.alpha += score * robot.alpha;

                total_score += score;
            }

            output_robot.position /= total_score;
            // TODO Angle average
            output_robot.velocity /= total_score;
            output_robot.omega /= total_score;
            output_robot.acceleration /= total_score;
            output_robot.alpha /= total_score;
        }
    }

    return yellow_robots_estimates;
}

std::array<std::optional<Robot>, 16> World::get_blue_robots_estimate()
{
    using RobotWithScore = std::pair<Robot, double>;
    std::array<std::vector<RobotWithScore>, 16> blue_robots_with_scores;

    // Grab estimates from camera
    for (auto & camera_pair : cameras)
    {
        std::array<std::optional<RobotWithScore>, 16> possible_blue_robots_with_score
            = camera_pair.second.get_blue_robot_estimates_with_score();

        for (size_t blue_id = 0; blue_id < 16; blue_id++)
        {
            if (possible_blue_robots_with_score.at(blue_id).has_value())
            {
                blue_robots_with_scores.at(blue_id).emplace_back(
                    possible_blue_robots_with_score.at(blue_id).value());
            }
        }
    }

    // Try to merge them together
    // return nullopt if there are no measurements
    std::array<std::optional<Robot>, 16> blue_robots_estimates;
    for (size_t blue_id = 0; blue_id < 16; blue_id++)
    {
        if (blue_robots_with_scores.empty())
        {
            blue_robots_estimates.at(blue_id) = std::nullopt;
        }
        else
        {
            // Merge robots based on weighted average of their scores
            blue_robots_estimates.at(blue_id) = Robot();

            auto & output_robot = blue_robots_estimates.at(blue_id).value();
            double total_score = 0.0;

            for (const auto & robot_with_score : blue_robots_with_scores.at(blue_id))
            {
                const Robot & robot = std::get<0>(robot_with_score);
                const double & score = std::get<1>(robot_with_score);
                output_robot.position += score * robot.position;
                // TODO Angle average
                output_robot.velocity += score * robot.velocity;
                output_robot.omega += score * robot.omega;
                output_robot.acceleration += score * robot.acceleration;
                output_robot.alpha += score * robot.alpha;

                total_score += score;
            }

            output_robot.position /= total_score;
            // TODO Angle average
            output_robot.velocity /= total_score;
            output_robot.omega /= total_score;
            output_robot.acceleration /= total_score;
            output_robot.alpha /= total_score;
        }
    }

    return blue_robots_estimates;
}