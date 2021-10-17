#pragma once

// Multiple kalman filters each with a different model

// Ball with rolling accel
// Ball with sliding accel
// Ball with static velocity
// Ball with bounce velocity in estimate direction
// Ball with slow kick robot facing direction
// Ball with medium kick robot facing direction
// Ball with fast kick robot facing direciton

// Robot with constant accel

#include "filters/kalman_filter.hpp"
#include "types/models.hpp"
#include "generators/model_input_generator.hpp"
#include "generators/transmission_probability_generator.hpp"

#include <Eigen/Dense>

#include <map>
#include <memory>
#include <vector>

class InteractingMultipleModelFilter {
public:
  void setup(
    const KalmanFilter& base_model,
    std::vector<Models::ModelType> model_types,
    std::shared_ptr<ModelInputGenerator> model_input_generator,
    std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator);

  InteractingMultipleModelFilter clone();

  void predict();
  void update(const Eigen::VectorXd & measurement);

  Eigen::VectorXd get_state_estimate() const;

  double get_potential_measurement_error(const Eigen::VectorXd & measurement);

private:
  void update_mu(const Eigen::VectorXd & zt);

  /**
   * Calculates the PDF of a multivariate normal distribution
   *
   * @param X Test point
   * @param mu mean
   * @param sigma variance
   *
   * @return PDF at test point
   */
  static double normal_multivariate_distribution_pdf(Eigen::VectorXd x, Eigen::VectorXd mu, Eigen::MatrixXd sigma);

  std::vector<Models::ModelType> model_types; // List of models 
  std::map<Models::ModelType, KalmanFilter> models; // Kalman filter representing ModelType
  std::map<Models::ModelType, double> mu; // ~= Probability of being in model ModelType

  unsigned int frames_since_last_update = 0;
  unsigned int updates_until_valid_track = 10;

  std::shared_ptr<ModelInputGenerator> model_input_generator;
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator;
};