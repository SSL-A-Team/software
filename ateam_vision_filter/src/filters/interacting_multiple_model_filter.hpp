// Copyright 2021 A Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef FILTERS__INTERACTING_MULTIPLE_MODEL_FILTER_HPP_
#define FILTERS__INTERACTING_MULTIPLE_MODEL_FILTER_HPP_

// Multiple kalman filters each with a different model

// Ball with rolling accel
// Ball with sliding accel
// Ball with static velocity
// Ball with bounce velocity in estimate direction
// Ball with slow kick robot facing direction
// Ball with medium kick robot facing direction
// Ball with fast kick robot facing direciton

// Robot with constant accel

#include <Eigen/Dense>

#include <map>
#include <memory>
#include <vector>

#include "filters/kalman_filter.hpp"
#include "types/models.hpp"
#include "generators/model_input_generator.hpp"
#include "generators/transmission_probability_generator.hpp"

class InteractingMultipleModelFilter
{
public:
  void setup(
    const KalmanFilter & base_model,
    std::vector<Models::ModelType> model_types,
    std::shared_ptr<ModelInputGenerator> model_input_generator,
    std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator);

  /**
   * Clone filter and set state estimate
   *
   * @note Clones all current tracks and covariance estimates. Only state estimates get changed
   * to new values
   */
  InteractingMultipleModelFilter clone(const Eigen::VectorXd & measurement);

  /**
   * Step models forward one time step
   */
  void predict();

  /**
   * Update models with new measurement
   */
  void update(const Eigen::VectorXd & measurement);

  /**
   * @return Best estimation of state (special weighted average of model's xhat)
   */
  Eigen::VectorXd get_state_estimate() const;

  Eigen::VectorXd get_position_estimate() const;

  /**
   * @return True if this filter has been updated at a reasonable regularly-ness
   */
  bool has_been_updated_regularly() const;

  /**
   * @return Score representing how much this filter has been updated
   */
  double get_validity_score() const;

  /**
   * @return Score of how far off a potential measurement error is
   */
  double get_potential_measurement_error(const Eigen::VectorXd & measurement);

//private:
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
  static double normal_multivariate_distribution_pdf(
    Eigen::VectorXd x, Eigen::VectorXd mu,
    Eigen::MatrixXd sigma);

  std::vector<Models::ModelType> model_types;  // List of models
  std::map<Models::ModelType, KalmanFilter> models;  // Kalman filter representing ModelType
  std::map<Models::ModelType, double> mu;  // ~= Probability of being in model ModelType

  double alpha = 0.9;  // How much to weight old updates when getting average frequency
  double relative_update_frequency = 0.5;
  unsigned int updates_until_valid_track = 10;
  double regularly_updated_frequncy_cutoff = 0.1;

  std::shared_ptr<ModelInputGenerator> model_input_generator;
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator;
};

#endif  // FILTERS__INTERACTING_MULTIPLE_MODEL_FILTER_HPP_
