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

#include "behavior/behavior_evaluator.hpp"

#include <functional>
#include <vector>
#include <cmath>

#include <ateam_common/parameters.hpp>
#include <ateam_common/game_state_listener.hpp>
#include <ateam_common/team_color_listener.hpp>

#include "dag_generation/halt.hpp"
#include "dag_generation/defend.hpp"
#include "dag_generation/shoot.hpp"

CREATE_PARAM(double, "behavior_evaluator/", kRotationSpeed, 0.005);

BehaviorEvaluator::BehaviorEvaluator(BehaviorRealization & behavior_realization)
: behavior_realization(behavior_realization) {}

DirectedGraph<BehaviorGoal> BehaviorEvaluator::get_best_behaviors(const World & world)
{
  // Get important world info
  ateam_common::TeamColorListener::TeamColor our_team_color_ = world.referee_info.our_team_color;
  ateam_common::GameCommand current_command_ = world.referee_info.running_command;

  // Check if we need to halt
  if (current_command_ == ateam_common::GameCommand::Halt) {
        return generate_halt(world);
  }

  // Check if we need to stop

  // Check for kickoff - indicates setup, normal start means to actually execute the play
  // If our kickoff, setup for basic kickoff play (shoot)
  // If their kickoff, setup for basic defensive play

  // Check for free kick - same as kickoff, means we should setup not actually kick
  // Call defense or offense with extra parameters

  // Check if we are in a penalty

  // If we get a normal start, check what the previous play was 
  // If it was either kickoff or free kick



  // If we can't identify what is going on, default to halt
  return generate_halt(world);
}
