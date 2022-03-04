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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "behavior/behavior.hpp"
#include "behavior/behavior_feedback.hpp"
#include "behavior/behavior_evaluator.hpp"
#include "behavior/behavior_executor.hpp"
#include "behavior/behavior_realization.hpp"
#include "types/world.hpp"
#include "util/directed_graph.hpp"

namespace ateam_ai
{

class ATeamAINode : public rclcpp::Node
{
public:
  explicit ATeamAINode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ateam_ai", options), evaluator(realization), executor(realization)
  {
    DirectedGraph<Behavior> current_behaviors;

    while (true) {
      current_behaviors = evaluator.get_best_behaviors(world);
      executor.execute_behaviors(current_behaviors, world);
    }
  }

private:
  BehaviorRealization realization;
  BehaviorEvaluator evaluator;
  BehaviorExecutor executor;
  World world;
};

}  // namespace ateam_ai

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_ai::ATeamAINode)
