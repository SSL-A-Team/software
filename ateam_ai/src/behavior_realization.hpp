#include <vector>
#include <queue>

#include "behavior.hpp"
#include "behavior_feedback.hpp"
#include "behavior_execution.hpp"

class BehaviorRealization {
public:
  /**
   * Given a set of behaviors, return trajectories and timings of all the behaviors expected for execution
   *
   * @note Complicated version that's ideal
   */
  DirectedGraph<BehaviorFeedback> realize_behaviors(const DirectedGraph<Behavior> & behaviors) {
    std::vector<std::size_t> root_nodes = behaviors.get_root_nodes();

    // Sort root nodes based on priority, highest priority first

    for (auto root_node : root_nodes) {
      //   b - d
      //  /
      // a
      //  \
      //   c - e
      //     \
      //       f
      // Given the directed graph above
      // We need to order the behaviors as a function of time
      // to figure out which robots may be already assigned by a node in the other side
      // Then, as a function of time assign robots based on availability
      //
      // We can't actually do the conversion to the function of time because the time each behavior
      // takes to complete is a function of the assigned robot, which is a function of the robot availability, which requires knowning
      // the behaviors as a function of time
      //
      //
      // We need to progress through the graph and assign as we go
      // Starting with the root node, we know...
      //  - Robot positions
      // We then greedily assign a robot to the root node based on earliest behavior completion time
      // Lets say robot 1 is assigned and will complete at 5 seconds.
      //
      // Moving on to the next two nodes, b and c. We know...
      //  - Robot positions (robot 1 at behavior position a, all others at start positions)
      //  - Robot availibilty (robot 1 free at 5 seconds, all others at 0 seconds)
      // We then independently do assignment for node b and c
      // Assignment is based on earliest completion time.
      //  - Time for robot 1 to go from position a to position b or c + 5 seconds
      //  - Time for all other robots to go from start positions to b or c
      // Based on whichever node (b or c) completes earliest, we then assign that robot
      // It should be said that the behavior cannot complete before the previous node completes
      // Lets say robot 2 is assigned and will complete at 10 seconds for node c
      //
      // Moving on to the next children nodes available: b, e, and f. We know...
      //  - Robot positions (robot 1 at a, robot 2 at c, all others at  start positions)
      //  - Robot availibility (robot 1 free at 5 seconds, robot 2 at 10 seconds, all others 0 seconds)
      // Repeat the process as before

      // We may need to have multiple different numbers in terms of "behavior completed"
      // and when the next behavior can "start"
      //
      // There are two be sets of start/end times to think about
      // First is the start/end time of ball interactions (and ball travel time between)
      //   This limits the start/end time of behaviors next to each other
      // Second is the start/end time of robot trajectory control
      //   This limits role assignment's use of previously assigned robots
    }

    return DirectedGraph<BehaviorFeedback>();
  }
  
  /**
   * Given a set of behaviors, return trajectories and timings of all the behaviors expected for execution
   *
   * @note Simple version that's easy to implement initially
   */
  DirectedGraph<BehaviorFeedback> realize_behaviors(const DirectedGraph<Behavior> & behaviors) {
    std::vector<std::size_t> root_nodes = behaviors.get_root_nodes();

    // Sort root nodes based on priority, highest priority first

    for (auto root_node : root_nodes) {
      // Moving through every node in the graph, assign it a not already assigned robot
      // If we run out of robots before running out of required behaviors, fail
    }

    return DirectedGraph<BehaviorFeedback>();
  }
};