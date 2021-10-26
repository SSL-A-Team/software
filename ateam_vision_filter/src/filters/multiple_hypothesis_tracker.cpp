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

#include <vector>

void MultipleHypothesisTracker::set_base_track(const InteractingMultipleModelFilter & base_track)
{
  this->base_track = base_track;
}

void MultipleHypothesisTracker::predict()
{
  for (auto & track : tracks) {
    track.predict();
  }
}

void MultipleHypothesisTracker::update(const std::vector<Eigen::VectorXd> & measurements)
{
  // Data association
  // minimize assignment cost where cost is different between each
  // model's predicted location and the measurement location
  //
  // Form is a bipartite graph assignmnet problem with 1 supply per measurement,
  // and 1 sink per track
}

void MultipleHypothesisTracker::life_cycle_management()
{
  // Life cycle management
  // Anything that's missed too many should be removed
}
