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

#include <boost/graph/successive_shortest_path_nonnegative_weights.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <map>
#include <set>
#include <utility>
#include <vector>
#include <iostream>

void MultipleHypothesisTracker::set_base_track(const InteractingMultipleModelFilter & base_track)
{
  this->base_track = base_track;
}

void MultipleHypothesisTracker::update(const std::vector<Eigen::VectorXd> & measurements)
{
  // Data association
  // minimize assignment cost where cost is different between each
  // model's predicted location and the measurement location
  //
  // Form is a bipartite graph assignmnet problem with 1 supply per measurement,
  // and 1 sink per track

  using adjacency_list_traits = boost::adjacency_list_traits<boost::vecS, boost::vecS,
      boost::directedS>;

  using graph_t = boost::adjacency_list<
    boost::vecS,
    boost::vecS,
    boost::directedS,
    boost::no_property,
    boost::property<boost::edge_capacity_t, unsigned int,
    boost::property<boost::edge_residual_capacity_t, unsigned int,
    boost::property<boost::edge_reverse_t, adjacency_list_traits::edge_descriptor,
    boost::property<boost::edge_weight_t, double>>>>>;
  using edge_capacity_list_t = boost::property_map<graph_t, boost::edge_capacity_t>::type;
  using edge_residual_capacity_list_t = boost::property_map<graph_t,
      boost::edge_residual_capacity_t>::type;
  using edge_weight_list_t = boost::property_map<graph_t, boost::edge_weight_t>::type;
  using edge_reverse_list_t = boost::property_map<graph_t, boost::edge_reverse_t>::type;

  adjacency_list_traits::vertex_descriptor source, sink;
  graph_t graph;
  edge_capacity_list_t edge_capacity_list = boost::get(boost::edge_capacity, graph);
  edge_weight_list_t edge_weight_list = boost::get(boost::edge_weight, graph);
  edge_reverse_list_t edge_reverse_list = boost::get(boost::edge_reverse, graph);

  //
  //     a - 1
  //   /   X   \.
  // s - b - 2 - e
  //   \   X   /
  //     c - 3
  //
  // Where s is the source, e is the sink
  // The left set (a,b,c) and right set (1,2,3) is fully connected
  // All edges have 1 flow representing 1 to 1 assignment of measurements to tracks
  // The cost between the left set and right set represent the distance between

  source = boost::add_vertex(graph);
  sink = boost::add_vertex(graph);

  std::map<adjacency_list_traits::vertex_descriptor,
    const Eigen::VectorXd &> vertex_to_measurement;
  for (size_t i = 0; i < measurements.size(); i++) {
    vertex_to_measurement.insert({boost::add_vertex(graph), measurements.at(i)});
  }

  std::map<adjacency_list_traits::vertex_descriptor,
    InteractingMultipleModelFilter &> vertex_to_track;
  for (size_t i = 0; i < tracks.size(); i++) {
    vertex_to_track.insert({boost::add_vertex(graph), tracks.at(i)});
  }

  std::map<
    adjacency_list_traits::edge_descriptor,
    std::pair<
      adjacency_list_traits::vertex_descriptor,
      adjacency_list_traits::vertex_descriptor>> forward_edge_to_vertex_pair;
  auto add_edge_to_graph = [&](auto & source_vertex, auto & sink_vertex, double weight) {
      adjacency_list_traits::edge_descriptor forward_edge, backward_edge;
      bool is_valid = true;
      unsigned int capacity = 1;
      boost::tie(forward_edge, is_valid) = boost::add_edge(
        boost::vertex(
          source_vertex,
          graph),
        boost::vertex(sink_vertex, graph), graph);

      assert(is_valid);  // Fails if edge already exists

      edge_capacity_list[forward_edge] = capacity;
      edge_weight_list[forward_edge] = weight;

      boost::tie(backward_edge, is_valid) = boost::add_edge(
        boost::vertex(
          sink_vertex,
          graph),
        boost::vertex(source_vertex, graph), graph);

      assert(is_valid);  // Fails if edge already exists

      edge_capacity_list[backward_edge] = 0;
      edge_weight_list[backward_edge] = -weight;

      edge_reverse_list[forward_edge] = backward_edge;
      edge_reverse_list[backward_edge] = forward_edge;

      forward_edge_to_vertex_pair[forward_edge] = std::make_pair(source_vertex, sink_vertex);
    };

  // Add all source to first set
  for (auto & vertex_measurement_pair : vertex_to_measurement) {
    add_edge_to_graph(source, vertex_measurement_pair.first, 0.0);
  }

  // Add all second set to sink
  for (auto & vertex_track_pair : vertex_to_track) {
    add_edge_to_graph(vertex_track_pair.first, sink, 0.0);
  }

  // We don't actually care about the source -> * and * -> sink so just clear edge to vertex map
  forward_edge_to_vertex_pair.clear();

  // Add cost for every single combination of measurement to track
  for (const auto & vertex_measurement_pair : vertex_to_measurement) {
    for (auto & vertex_track_pair : vertex_to_track) {
      double dist =
        (vertex_measurement_pair.second - vertex_track_pair.second.get_position_estimate()).norm();
      add_edge_to_graph(vertex_measurement_pair.first, vertex_track_pair.first, dist);
    }
  }

  boost::successive_shortest_path_nonnegative_weights(graph, source, sink);

  std::set<adjacency_list_traits::vertex_descriptor> unassigned_measurements;
  for (const auto & vertex_measurement_pair : vertex_to_measurement) {
    unassigned_measurements.insert(vertex_measurement_pair.first);
  }

  // Note: residual capacity so 1 is not used, 0 is used
  edge_residual_capacity_list_t edge_residual_capacity_list = boost::get(
    boost::edge_residual_capacity, graph);
  for (const auto & forward_edge_vertex_pair : forward_edge_to_vertex_pair) {
    unsigned int used_flow = 1 - edge_residual_capacity_list[forward_edge_vertex_pair.first];

    // Assigned edge
    if (used_flow == 1) {
      auto & source_measurement_vertex = forward_edge_vertex_pair.second.first;
      auto & sink_track_vertex = forward_edge_vertex_pair.second.second;

      const auto & measurement = vertex_to_measurement.at(source_measurement_vertex);
      auto & track = vertex_to_track.at(sink_track_vertex);

      // Only add the measurement if they're within some range of the track
      // since measurements aren't super consistent
      if ((measurement - track.get_position_estimate()).norm() < 1) {
        track.update(measurement);

        unassigned_measurements.erase(source_measurement_vertex);
      }
    }
  }

  // For any leftover measurement, create a new track
  for (const auto & vertex_measurement : unassigned_measurements) {
    const auto & measurement = vertex_to_measurement.at(vertex_measurement);
    tracks.emplace_back(base_track.clone(measurement));
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

  // Otherwise return best scoring track
  size_t best_idx = 0;
  double best_score = 0.0;
  for (size_t track_id = 0; track_id < tracks.size(); track_id++) {
    if (best_score < tracks.at(track_id).get_validity_score()) {
      best_score = tracks.at(track_id).get_validity_score();
      best_idx = track_id;
    }
  }

  return std::make_pair(tracks.at(best_idx).get_state_estimate(), best_score);
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
