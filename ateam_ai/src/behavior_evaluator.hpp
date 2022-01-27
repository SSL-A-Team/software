#include <vector>

#include "behavior.hpp"
#include "behavior_feedback.hpp"
#include "behavior_realization.hpp"

class BehaviorEvaluator {
public:
  void get_best_behaviors(const std::vector<BehaviorFeedback> & previous_behavior_feedback,
                          std::vector<Behavior> & behavior_out) {
    //
    // Do preprocessing on world state to get important metrics like possession
    //

    //
    // Setup different behavior options
    //
    std::vector<Behavior> three_one_touch_shot;
    Behavior initial_pass_start;
    initial_pass_start.type = Behavior::Type::MovingKick;
    Behavior first_receiver;
    first_receiver.type = Behavior::Type::OneTouchReceiveKick;
    first_receiver.priority = Behavior::Priority::Required;
    // first_receiver.position = {0, 0};
    // first_receiver.target = {10, 10};
    Behavior second_receiver;
    second_receiver.type = Behavior::Type::OneTouchReceiveKick;
    second_receiver.priority = Behavior::Priority::Required;
    // second_receiver.position = {10, 10};
    // second_receiver.target = {-10, -10};
    Behavior final_receiver_shot;
    final_receiver_shot.type = Behavior::Type::OneTouchShot;
    final_receiver_shot.priority = Behavior::Priority::Required;
    // final_receiver_shot.position = {-10, -10};
    // final_receiver_shot.target = Field::TheirGoal;

    three_one_touch_shot.push_back(initial_pass_start);
    three_one_touch_shot.push_back(second_receiver);
    three_one_touch_shot.push_back(final_receiver_shot);


    std::vector<Behavior> direct_shot;
    Behavior initial_capture;
    initial_capture.type = Behavior::Type::GetBall;
    initial_capture.priority = Behavior::Priority::Required;
    Behavior shot;
    shot.type = Behavior::Type::Shot;
    initial_capture.priority = Behavior::Priority::Required;

    direct_shot.push_back(initial_capture);
    direct_shot.push_back(shot);

    //
    // See how that combination of behaviors would be planned and executed
    //
    BehaviorRealization behavior_realization; // This should be one class higher and passed in so stuff can be chached

    std::vector<BehaviorFeedback> three_one_touch_shot_feedback;
    behavior_realization.realize_behaviors(three_one_touch_shot, three_one_touch_shot_feedback);
    // Move the feedback timing to the behavior specification
    // for [behavior, behavior_feedback] : three_one_touch_shot, three_one_touch_shot_feedback) {
    //   behavior.start_time = behavior_feedback.start_time;
    //   behavior.completion_time = behavior_feeback.completion_time
    // }

    std::vector<BehaviorFeedback> direct_shot_feedback;
    behavior_realization.realize_behaviors(direct_shot, direct_shot_feedback);
    // Move the feedback timing to the behavior specification
    // for [behavior, behavior_feedback] : direct_shot_feedback, direct_shot_feedback) {
    //   behavior.start_time = behavior_feedback.start_time;
    //   behavior.completion_time = behavior_feeback.completion_time
    // }

    //
    // Choose main behavior
    //

    // choose direct shot because score chance is better or
    // maybe the total behavior completetion time is short
    // or maybe the other one can't be completed due to number of robots
    behavior_out = direct_shot;

    //
    // Add background behaviors
    //

    Behavior goalie;
    goalie.type = Behavior::Type::MoveToPoint;
    goalie.priority = Behavior::Priority::Medium;
    // goalie.position = Field::OurGoal.center();
    Behavior forward;
    forward.type = Behavior::Type::MoveToPoint;
    forward.priority = Behavior::Priority::Low;
    // forward.position = Field::TheirGoal.center();
    Behavior left_defender;
    left_defender.type = Behavior::Type::MoveToPoint;
    left_defender.priority = Behavior::Priority::Medium;
    // left_defender.position = {5, 5};
    Behavior right_defender;
    right_defender.type = Behavior::Type::MoveToPoint;
    right_defender.priority = Behavior::Priority::Medium;
    // right_defender.position = {5, -5};

    behavior_out.push_back(goalie);
    behavior_out.push_back(forward);
    behavior_out.push_back(left_defender);
    behavior_out.push_back(right_defender);
  }
};