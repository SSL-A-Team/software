#include <vector>

#include "behavior.hpp"
#include "behavior_feedback.hpp"
#include "behavior_execution.hpp"

class BehaviorRealization {
public:
  void realize_behaviors(const std::vector<Behavior> & behaviors,
                         std::vector<BehaviorFeedback> & behavior_feedbacks_out) {
    std::vector<BehaviorExecution> temp;
    realize_behaviors(behaviors, behavior_feedbacks_out, temp);
  }

  /**
   * @param behaviors List of behaviors seperated into two sections, all required behaviors are in time order,
   *  all non required behaviors are in importance order
   */
  void realize_behaviors(const std::vector<Behavior> & behaviors,
                         std::vector<BehaviorFeedback> & behavior_feedbacks_out,
                         std::vector<BehaviorExecution> & behavior_execution_out) {
    // Moving through each behavior in time starting at highest priority set
    //  Note: Since we are moving from high priority to low priority,
    //    the required behaviors will be assigned consistently without needing to worry about
    //    different background behaviors for other robots
    //
    // Assign a robot to that behavior
    //  Note: Some behaviors may use the same robot (GetBall -> PivotKick)
    //  Note: Some earlier behaviors in the time series may release robots so we can reuse them
    //
    // Calculate trajectory for that robot to complete the behavior
    // Figure out the earliest time a robot can start the behavior
    // Figure out how long the behavior should take
  }
};