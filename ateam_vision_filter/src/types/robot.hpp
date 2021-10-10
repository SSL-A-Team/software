#include <Eigen/Dense>

struct Robot {
    Eigen::Vector2d position;
    double theta;

    Eigen::Vector2d velocity;
    double omega;

    Eigen::Vector2d acceleration;
    double alpha;
}