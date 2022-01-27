#include <vector>

#include "behavior.hpp"
#include "behavior_execution.hpp"
#include "behavior_feedback.hpp"
#include "behavior_realization.hpp"

class BehaviorExecutor {
public:
  void execute_behaviors(const std::vector<Behavior> & behaviors,
                         std::vector<BehaviorFeedback> & behavior_feedbacks_out) {
    //
    // Grab trajectories for everything
    //
    BehaviorRealization behavior_realization; // This should be one class higher and passed in so stuff can be chached
    std::vector<BehaviorExecution> behavior_executions;
    behavior_realization.realize_behaviors(behaviors, behavior_feedbacks_out, behavior_executions);

    // Can filter out behaviors that we don't need to start moving towards yet
    // Can filter out behaviors that should be done already

    //
    // Follow trajectories
    //

    // send commands down to motion control
    // edit times based on actual location inside trajectory
  }
};