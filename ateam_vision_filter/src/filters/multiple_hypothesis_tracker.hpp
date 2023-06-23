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

#ifndef FILTERS__MULTIPLE_HYPOTHESIS_TRACKER_HPP_
#define FILTERS__MULTIPLE_HYPOTHESIS_TRACKER_HPP_

// Matches measurements to filters
// Creates tracks as needed
// Removes tracks as needed
// Returns best track

#include <Eigen/Dense>

#include <map>
#include <utility>
#include <vector>

#include <ateam_msgs/msg/vision_mht_state.hpp>

#include "filters/interacting_multiple_model_filter.hpp"

class MultipleHypothesisTracker
{
public:
  using StateWithScore = std::pair<Eigen::VectorXd, double>;

  void set_base_track(const InteractingMultipleModelFilter & base_track);

  void update(const std::vector<Eigen::VectorXd> & measurements);
  void predict();

  std::optional<StateWithScore> get_state_estimate() const;

  /**
   * @return ROS2 msg containing the current internal state
   */
  ateam_msgs::msg::VisionMHTState get_vision_mht_state() const;

private:
//   using adjacency_list_traits = boost::adjacency_list_traits<boost::vecS, boost::vecS,
//       boost::directedS>;

//   using graph_t = boost::adjacency_list<
//     boost::vecS,
//     boost::vecS,
//     boost::directedS,
//     boost::no_property,
//     boost::property<boost::edge_capacity_t, unsigned int,
//     boost::property<boost::edge_residual_capacity_t, unsigned int,
//     boost::property<boost::edge_reverse_t, adjacency_list_traits::edge_descriptor,
//     boost::property<boost::edge_weight_t, double>>>>>;
//   using edge_capacity_list_t = boost::property_map<graph_t, boost::edge_capacity_t>::type;
//   using edge_residual_capacity_list_t = boost::property_map<graph_t,
//       boost::edge_residual_capacity_t>::type;
//   using edge_weight_list_t = boost::property_map<graph_t, boost::edge_weight_t>::type;
//   using edge_reverse_list_t = boost::property_map<graph_t, boost::edge_reverse_t>::type;
//   using forward_edge_to_vertex_pair_t = std::map<
//     adjacency_list_traits::edge_descriptor,
//     std::pair<
//       adjacency_list_traits::vertex_descriptor,
//       adjacency_list_traits::vertex_descriptor>>;

//   void add_edge_to_graph(
//     graph_t & graph,
//     edge_capacity_list_t & edge_capacity_list,
//     edge_weight_list_t & edge_weight_list,
//     edge_reverse_list_t & edge_reverse_list,
//     forward_edge_to_vertex_pair_t & forward_edge_to_vertex_pair,
//     adjacency_list_traits::vertex_descriptor source_vertex,
//     adjacency_list_traits::vertex_descriptor sink_vertex,
//     double weight);

  void life_cycle_management();

  InteractingMultipleModelFilter base_track;

  std::vector<InteractingMultipleModelFilter> tracks;
};

#endif  // FILTERS__MULTIPLE_HYPOTHESIS_TRACKER_HPP_
