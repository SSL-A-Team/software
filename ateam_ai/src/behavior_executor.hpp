#include "behavior.hpp"
#include "behavior_execution.hpp"
#include "behavior_feedback.hpp"
#include "behavior_realization.hpp"

class BehaviorExecutor {
public:
  void execute_behaviors(const DirectedGraph<Behavior> & behaviors,
                         BehaviorRealization & behavior_realization) {
    //
    // Grab trajectories for everything
    //
    DirectedGraph<BehaviorFeedback> behavior_feedback = behavior_realization.realize_behaviors(behaviors);

    // Ideally, convert the behavior graph into a per robot list of trajectories as a function of time
    //    https://help.perforce.com/visualization/jviews/documentation/userman/gantt/images/gen_gantt_default.png
    // each row is a robot
    // columns are a function of time
    // a trajectory fills section of columns for a single robot based on it's start/end time

    //
    // Replan course trajectories as we approach their start time
    //

    // Long term planning (>10 seconds) is not valid due to the speed of robots
    // as we approach some of the later behaviors in time, we need to replan them using better planners
    // to dodge these obsticles and everything

    //
    // Follow trajectories
    //

    // send commands down to motion control
  }
};