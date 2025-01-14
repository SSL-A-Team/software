// Copyright 2024 A Team
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

#include "spatial_evaluator.hpp"
#include <ranges>
#include "layers/distance_down_field.hpp"
#include "layers/distance_from_their_bots.hpp"
#include "layers/line_of_sight_ball.hpp"
#include "layers/line_of_sight_their_goal.hpp"
#include "maps/receiver_position_quality.hpp"
#include "maps/test_map.hpp"

namespace ateam_kenobi::spatial
{

SpatialEvaluator::SpatialEvaluator()
{
  layer_factories_ = {
    std::make_shared<layers::DistanceDownField>(),
    std::make_shared<layers::DistanceFromTheirBots>(),
    std::make_shared<layers::LineOfSightBall>(),
    std::make_shared<layers::LineOfSightTheirGoal>()
  };
  map_factories_ = {
    std::make_shared<maps::TestMap>()
  };
}

void SpatialEvaluator::Update(World & world)
{
  BuildLayers(world);
  BuildMaps(world);
}

void SpatialEvaluator::BuildLayers(World & world)
{
  for(auto & factory : layer_factories_) {
    factory->Build(layers_[factory->GetName()], world);
  }
}

void SpatialEvaluator::BuildMaps(World & world)
{
  for(auto & factory : map_factories_) {
    const auto & name = factory->GetName();
    auto & map = world.spatial_maps[name];
    if(map.name.empty()) {
      map.name = name;
    }
    factory->FillMap(map.data, world, layers_);
  }
}

} // namespace ateam_kenobi::spatial
