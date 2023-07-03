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

#include "filters/multiple_hypothesis_tracker.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <utility>
#include <vector>
#include <iostream>
#include <limits>

#include "ateam_common/assignment.hpp"

void MultipleHypothesisTracker::set_base_track(const InteractingMultipleModelFilter & base_track)
{
  this->base_track = base_track;
}

void MultipleHypothesisTracker::update(const std::vector<Eigen::VectorXd> & measurements)
{
  // Data association
  // minimize assignment cost where cost is different between each
  // model's predicted location and the measurement location
  // in other words the assignment matricies costs represent the
  // distance between each existing track and each measurement

  Eigen::MatrixXd costs = Eigen::MatrixXd::Constant(
    tracks.size(),
    measurements.size(), std::numeric_limits<double>::infinity());
  for (size_t i = 0; i < tracks.size(); i++) {
    for (size_t j = 0; j < measurements.size(); j++) {
      costs(i, j) = (measurements.at(j) - tracks.at(i).get_position_estimate()).norm();
    }
  }

  std::unordered_map assignment = ateam_common::assignment::optimize_assignment(costs);
  std::vector<bool> used_measurements(measurements.size(), false);

  // iterate over assigned measurement track pairs and update the track
  for (const auto & [track_idx, measurement_idx] : assignment) {
    // Assigned edge
    used_measurements.at(measurement_idx) = true;

    const auto & measurement = measurements.at(measurement_idx);
    auto & track = tracks.at(track_idx);

    // Only add the measurement if they're within some range of the track
    // since measurements aren't super consistent
    // if ((measurement - track.get_position_estimate()).norm() < 1) {
    track.update(measurement);
  }

  // For any leftover measurements, create a new track
  for (size_t i = 0; i < used_measurements.size(); i++) {
    if (used_measurements.at(i) == false) {
      const auto & measurement = measurements.at(i);
      tracks.emplace_back(base_track.clone(measurement));
    }
  }
}

void MultipleHypothesisTracker::predict()
{
  life_cycle_management();

  for (auto & track : tracks) {
    track.predict();
  }
}

std::optional<MultipleHypothesisTracker::StateWithScore> MultipleHypothesisTracker::
get_state_estimate() const
{
  // Only return values if we have tracks
  if (tracks.empty()) {
    return std::nullopt;
  }

  const auto best_track = std::min_element(
    tracks.begin(), tracks.end(), [](const auto & a, const auto & b) {
      return a.get_validity_score() > b.get_validity_score();
    });

  return std::make_pair(best_track->get_state_estimate(), best_track->get_validity_score());
}

ateam_msgs::msg::VisionMHTState MultipleHypothesisTracker::get_vision_mht_state() const
{
  ateam_msgs::msg::VisionMHTState mht_state;
  for (const auto & track : tracks) {
    mht_state.tracks.push_back(track.get_vision_imm_state());
  }

  return mht_state;
}

void MultipleHypothesisTracker::life_cycle_management()
{
  // Anything that hasn't been updated regularly should be removed
  tracks.erase(
    std::remove_if(
      tracks.begin(), tracks.end(), [](const auto & track) {
        return !track.has_been_updated_regularly();
      }), tracks.end());
}
