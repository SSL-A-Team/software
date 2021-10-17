#include "filters/interacting_multiple_model_filter.hpp"

#include <algorithm>
#include <cmath>

void InteractingMultipleModelFilter::setup(
  const KalmanFilter& base_model,
  std::vector<Models::ModelType> model_types,
  std::shared_ptr<ModelInputGenerator> model_input_generator,
  std::shared_ptr<TransmissionProbabilityGenerator> transmission_probability_generator)
{
  this->model_types = model_types;
  this->model_input_generator = model_input_generator;
  this->transmission_probability_generator = transmission_probability_generator;

  for (const auto& model_type : model_types)
  {
    models[model_type] = base_model;
  }
}

void InteractingMultipleModelFilter::predict()
{
  for (auto& model_pair : models)
  {
    model_pair.second.predict(
      model_input_generator->get_model_input(
        model_pair.second.get_x_hat(),
        model_pair.first));
  }

  ++frames_since_last_update;
}

void InteractingMultipleModelFilter::update(const Eigen::VectorXd & measurement)
{
  update_mu(measurement);

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

void InteractingMultipleModelFilter::update_mu(const Eigen::VectorXd & zt)
{
  // Eq 14
  // https://arxiv.org/pdf/1912.00603.pdf
  //
  // Approximates the posterior state estimate
  // as a Gaussian mixture of that given by multiple filters
  // which can be weightred by mu
  // to produce the overall estimate

  double normalization_factor = 0.0;
  std::map<Models::ModelType, double> current_time_step_mu;

  for (const auto& model_type : model_types)
  {
  
    // Grab the probability of transitioning from all other models to this model
    // Weighted individually by the probability of being in that model last frame
    double probability_of_transition_to_model = 0;
    for (const auto& other_model_type : model_types)
    {
      double probability_transition_from_other_model_to_model =
        transmission_probability_generator->get_transmission_probability(
          models.at(model_type).get_x_hat(),
          other_model_type,
          model_type);
      probability_of_transition_to_model += mu.at(other_model_type) * probability_transition_from_other_model_to_model;
    }

    // Weight probability of being in this model by how well this model matches the measurement
    double probability_of_measurement_to_model = normal_multivariate_distribution_pdf(
      zt,
      models.at(model_type).get_y(),
      models.at(model_type).get_estimated_gaussian_variance());
    current_time_step_mu[model_type] = probability_of_measurement_to_model * probability_of_transition_to_model;
    normalization_factor += current_time_step_mu.at(model_type);
  }

  for (const auto& model_type : model_types) {
    mu.at(model_type) = current_time_step_mu.at(model_type) / normalization_factor;
  }
}

double InteractingMultipleModelFilter::normal_multivariate_distribution_pdf(
  Eigen::VectorXd x, Eigen::VectorXd mu, Eigen::MatrixXd sigma)
{
  // PDF equation at https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  return std::pow(2 * M_PI, -1 * x.size() / 2.0) *
         std::pow(sigma.determinant(), -1.0 / 2.0) *
         std::exp(-1.0 / 2.0 * (x - mu).transpose() * sigma.inverse() * (x - mu));
}