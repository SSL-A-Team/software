#ifndef UTIL__KICKOFF_DETECTOR_HPP_
#define UTIL__KICKOFF_DETECTOR_HPP_

#include <chrono>
#include <optional>

#include "types/world.hpp"
#include "util/directed_graph.hpp"


using namespace std::chrono_literals;

// I think this is officially the worst code ive written but im sick... Please someone rewrite this

/**
 *  Small class to be "ticked" with the world state and keep track of if we enter a kickoff and then when we exit (ball has moved 0.05 after normal_start is issued, 10 seconds pass, force start is issued)
 *
*/
class KickoffDetector {
public:

    enum class KickoffState {
        Ignore,
        PrepKickoff,
        Kickoff,
        Play,
    };
    // Ctors
    KickoffDetector() {};

    // Vars
    std::chrono::steady_clock::time_point resume_play_time {std::chrono::steady_clock::now()};
    ateam_common::GameCommand prev_command {ateam_common::GameCommand::Stop};
    KickoffState state {KickoffState::Ignore};
    std::optional<Eigen::Vector2d> maybe_kickoff_position {}; // optional in case ball cant be seen

    static constexpr double DIST_THRESHOLD {0.05};


    // maybe I should have just had this return original game states so the outside logic didnt have to depend on anything in here
    void update(World world) {
        // edge triggered
        ateam_common::GameCommand cur_command = world.referee_info.running_command;
        if (prev_command != cur_command) {
            // dont really even think we need to know what we came from just what we are in
            switch (cur_command) {
                case ateam_common::GameCommand::PrepareKickoffOurs:
                case ateam_common::GameCommand::PrepareKickoffTheirs:
                    // start timer
                    state = KickoffState::PrepKickoff;
                    resume_play_time = std::chrono::steady_clock::now() + 10s;
                    break;

                case ateam_common::GameCommand::NormalStart:
                    state = KickoffState::Kickoff;
                    break;

                // force play
                case ateam_common::GameCommand::ForceStart:
                    state = KickoffState::Play;
                    break;
                default:
                    state = KickoffState::Ignore;
            }
        }

        prev_command = cur_command;

        // level
        if ((state == KickoffState::PrepKickoff || state == KickoffState::Kickoff)
            && std::chrono::steady_clock::now() < resume_play_time) {
            // timeout go to play
            state = KickoffState::Play;
        }

        if (state == KickoffState::Kickoff) {
            auto maybe_ball = world.get_unique_ball();
            if (maybe_ball.has_value()){
                if (maybe_kickoff_position.has_value()) {
                    // if we are tracking a start position and a current ball and its further away than dist we are in play
                    if ((maybe_ball.value().pos - maybe_kickoff_position.value()).norm() > DIST_THRESHOLD) {
                        state = KickoffState::Play;
                    }
                } else {
                    // set the ball position if we dont have one yet during this kickoff
                    maybe_kickoff_position = maybe_ball.value().pos;
                }
            }
        } else {
            maybe_kickoff_position = std::nullopt;
        }

    }
}

#endif  // UTIL__KICKOFF_DETECTOR_HPP_