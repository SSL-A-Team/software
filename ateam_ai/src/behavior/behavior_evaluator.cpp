#include "behavior/behavior_evaluator.hpp"

#include "behavior/behavior_feedback.hpp"

BehaviorEvaluator::BehaviorEvaluator(BehaviorRealization & behavior_realization)
  : behavior_realization(behavior_realization) {}

DirectedGraph<Behavior> BehaviorEvaluator::get_best_behaviors()
{
  //
  // Do preprocessing on world state to get important metrics like possession
  //

  //
  // Setup different behavior options
  //

  DirectedGraph<Behavior> three_one_touch_shot;
  Behavior initial_pass_start{
    Behavior::Type::MovingKick,
    Behavior::Priority::Required,
    KickParam({0, 0})};
  Behavior first_receiver{
    Behavior::Type::OneTouchReceiveKick,
    Behavior::Priority::Required,
    ReceiveParam({0, 0}, {10, 10})};
  Behavior second_receiver{
    Behavior::Type::OneTouchReceiveKick,
    Behavior::Priority::Required,
    ReceiveParam({10, 10}, {-10, -10})};
  Behavior final_receiver_shot{
    Behavior::Type::OneTouchShot,
    Behavior::Priority::Required,
    ReceiveShotParam({-10, -10})};

  std::size_t parent = three_one_touch_shot.add_node(initial_pass_start);
  parent = three_one_touch_shot.add_node(first_receiver, parent);
  parent = three_one_touch_shot.add_node(second_receiver, parent);
  parent = three_one_touch_shot.add_node(final_receiver_shot, parent);


  DirectedGraph<Behavior> direct_shot;
  Behavior shot{
    Behavior::Type::Shot,
    Behavior::Priority::Required,
    ShotParam()};

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

  Behavior goalie{
    Behavior::Type::MoveToPoint,
    Behavior::Priority::Medium,
    MoveParam({0, 0})};  // Field::OurGoal.center();
  Behavior forward{
    Behavior::Type::MoveToPoint,
    Behavior::Priority::Low,
    MoveParam({10, 0})};  // Field::TheirGoal.center();
  Behavior left_defender{
    Behavior::Type::MoveToPoint,
    Behavior::Priority::Medium,
    MoveParam({5, 5})};
  Behavior right_defender{
    Behavior::Type::MoveToPoint,
    Behavior::Priority::Medium,
    MoveParam({5, -5})};

  behavior_out.add_node(goalie);
  behavior_out.add_node(forward);
  behavior_out.add_node(left_defender);
  behavior_out.add_node(right_defender);

  return behavior_out;
}