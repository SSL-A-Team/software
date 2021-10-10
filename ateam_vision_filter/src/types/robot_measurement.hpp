#include <Eigen/Dense>

struct RobotMeasurement {
    Eigen::Vector2d position;
    double theta;
}