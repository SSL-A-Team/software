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

#ifndef BEHAVIOR_EVALUATOR_HPP_
#define BEHAVIOR_EVALUATOR_HPP_

#include <vector>

#include "behavior.hpp"
#include "behavior_feedback.hpp"
#include "behavior_realization.hpp"
#include "directed_graph.hpp"

class BehaviorEvaluator
{
public:
  DirectedGraph<Behavior> get_best_behaviors(BehaviorRealization & behavior_realization)
  {
    //
    // Do preprocessing on world state to get important metrics like possession
    //

    //
    // Setup different behavior options
    //

    DirectedGraph<Behavior> three_one_touch_shot;
    Behavior initial_pass_start;
    initial_pass_start.type = Behavior::Type::MovingKick;
    initial_pass_start.priority = Behavior::Priority::Required;
    initial_pass_start.params = KickParam({0, 0});
    Behavior first_receiver;
    first_receiver.type = Behavior::Type::OneTouchReceiveKick;
    first_receiver.priority = Behavior::Priority::Required;
    first_receiver.params = ReceiveParam({0, 0}, {10, 10});
    Behavior second_receiver;
    second_receiver.type = Behavior::Type::OneTouchReceiveKick;
    second_receiver.priority = Behavior::Priority::Required;
    second_receiver.params = ReceiveParam({10, 10}, {-10, -10});
    Behavior final_receiver_shot;
    final_receiver_shot.type = Behavior::Type::OneTouchShot;
    final_receiver_shot.priority = Behavior::Priority::Required;
    final_receiver_shot.params = ReceiveShotParam({-10, -10});

    std::size_t parent = three_one_touch_shot.add_node(initial_pass_start);
    parent = three_one_touch_shot.add_node(first_receiver, parent);
    parent = three_one_touch_shot.add_node(second_receiver, parent);
    parent = three_one_touch_shot.add_node(final_receiver_shot, parent);


    DirectedGraph<Behavior> direct_shot;
    Behavior initial_capture;
    initial_capture.type = Behavior::Type::GetBall;
    initial_capture.priority = Behavior::Priority::Required;
    initial_capture.params = GetBallParam();
    Behavior shot;
    shot.type = Behavior::Type::Shot;
    shot.priority = Behavior::Priority::Required;
    shot.params = ShotParam();

    parent = direct_shot.add_node(initial_capture);
    parent = direct_shot.add_node(shot, parent);

    //
    // See how that combination of behaviors would be planned and executed
    //
    DirectedGraph<BehaviorFeedback> three_one_touch_shot_feedback =
      behavior_realization.realize_behaviors(three_one_touch_shot);
    DirectedGraph<BehaviorFeedback> direct_shot_feedback =
      behavior_realization.realize_behaviors(direct_shot);

    //
    // Choose main behavior
    //

    // choose direct shot because score chance is better or
    // maybe the total behavior completetion time is short
    // or maybe the other one can't be completed due to number of robots
    DirectedGraph<Behavior> behavior_out = direct_shot;

    //
    // Add background behaviors
    //

    Behavior goalie;
    goalie.type = Behavior::Type::MoveToPoint;
    goalie.priority = Behavior::Priority::Medium;
    goalie.params = MoveParam({0, 0});  // Field::OurGoal.center();
    Behavior forward;
    forward.type = Behavior::Type::MoveToPoint;
    forward.priority = Behavior::Priority::Low;
    forward.params = MoveParam({10, 0});  // Field::TheirGoal.center();
    Behavior left_defender;
    left_defender.type = Behavior::Type::MoveToPoint;
    left_defender.priority = Behavior::Priority::Medium;
    left_defender.params = MoveParam({5, 5});
    Behavior right_defender;
    right_defender.type = Behavior::Type::MoveToPoint;
    right_defender.priority = Behavior::Priority::Medium;
    right_defender.params = MoveParam({5, -5});

    behavior_out.add_node(goalie);
    behavior_out.add_node(forward);
    behavior_out.add_node(left_defender);
    behavior_out.add_node(right_defender);

    return behavior_out;
  }
};

#endif  // BEHAVIOR_EVALUATOR_HPP_
