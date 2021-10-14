#include "interacting_multiple_model_filter.hpp"

#include <algorithm>
#include <cmath>

InteractingMultipleModelFilter::InteractingMultipleModelFilter(
  const KalmanFilter& base_model,
  std::vector<Models::Ball::ModelType> model_types,
  std::shared_ptr<ModelInputGenerator> model_input_generator,
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator)
  : model_types(model_types), model_input_generator(model_input_generator), transmission_probability_generator(transmission_probability_generator)
{
  for (const auto& model_type : model_types)
  {
    models[model_type] = base_model;
  }
}

void InteractingMultipleModelFilter::predict()
{
  for (auto& model_pair : models)
  {
    model_pair.second.predict(model_input_generator->get_model_input(Ball(), model_pair.first));
  }

  ++frames_since_last_update;
}

void InteractingMultipleModelFilter::update(Eigen::VectorXd measurement)
{
  for (auto& model_pair : models)
  {
    model_pair.second.update(measurement);
  }

  frames_since_last_update = 0;

  if (updates_until_valid_track > 0)
  {
    --updates_until_valid_track;
  }
}

double InteractingMultipleModelFilter::get_potential_measurement_error(const Eigen::VectorXd & measurement)
{
  // Uses previous frame mu because we haven't gotten this frames measurements yet
  // This is because the MHT is using this to associate measurements to tracks

  // Eq 21/22
  // https://arxiv.org/pdf/1912.00603.pdf
  //
  // Because z - sum mu * F * H * x
  // we can expand to sum 1/m z  - mu * F * H * x
  // Since mu sums to 1, we can replace 1/m with mu
  // resulting in sum mu * ( z - F * H * x)
  // allowing us to push the z - F * H * x into the kalman filter
  // and not expose model data to this class

  Eigen::VectorXd potential_measurement_error;

  for (const auto& model_type : model_types)
  {
    potential_measurement_error += mu.at(model_type) * models.at(model_type).get_potential_measurement_error(measurement);
  }

  return potential_measurement_error.norm();
}

Eigen::VectorXd InteractingMultipleModelFilter::get_state_estimate() const
{
  // Requires update_mu to have been called for this frame

  // Eq 19
  // https://arxiv.org/pdf/1912.00603.pdf
  //
  // Best estimate is a weighted average of the individual models estimate
  Eigen::VectorXd x_bar;

  for (const auto& model_type : model_types)
  {
    x_bar += mu.at(model_type) * models.at(model_type).get_x_hat();
  }

  return x_bar;
}

void InteractingMultipleModelFilter::update_mu()
{
  // Eq 14
  // https://arxiv.org/pdf/1912.00603.pdf
  //
  // Approximates the posterior state estimate
  // as a Gaussian mixture of that given by multiple filters
  // which can be weightred by mu
  // to produce the overall estimate

  double normalization_factor = 0.0;
  std::map<Models::Ball::ModelType, double> current_time_step_mu;

  // Use standard gaussian mixture
  for (const auto& model_type : model_types)
  {
    double probability_of_model = 0;
    for (const auto& other_model_type : model_types)
    {
      double probability_transition_to_model =
        transmission_probability_generator->get_tranmission_probability(Ball(), other_model_type, model_type);
      probability_of_model += probability_transition_to_model * mu.at(other_model_type);
    }

    current_time_step_mu[model_type] = 0;// normal_distribution_pdf(zt, H*x, H * P * H.transpose() + R) * probability_of_model;
    normalization_factor += current_time_step_mu.at(model_type);
  }

  for (const auto& model_type : model_types) {
    mu.at(model_type) = current_time_step_mu.at(model_type) / normalization_factor;
  }
}

double InteractingMultipleModelFilter::normal_distribution_pdf(double x, double mean, double sigma) const
{
  return 1 / (sigma * sqrt(2.0 * M_PI)) * std::exp(-1.0 / 2.0 * std::pow((x - mean) / sigma, 2));
}