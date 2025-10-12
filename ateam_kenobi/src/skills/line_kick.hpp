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


#ifndef SKILLS__LINE_KICK_HPP_
#define SKILLS__LINE_KICK_HPP_

#include <ateam_common/robot_constants.hpp>
#include "kick_skill.hpp"
#include "core/types.hpp"
#include "core/types/robot_command.hpp"

namespace ateam_kenobi::skills
{

class LineKick : public KickSkill
{
public:
  enum class KickType
  {
    Kick,
    Chip
  };

  explicit LineKick(
    stp::Options stp_options,
    KickSkill::WaitType wait_type = KickSkill::WaitType::KickWhenReady);

  void Reset() override
  {
    KickSkill::Reset();
    state_ = State::MoveBehindBall;
  }

  void SetTargetPoint(ateam_geometry::Point point)
  {
    if (state_ != State::KickBall) {
      target_point_ = point;
    }
  }

  ateam_geometry::Point GetAssignmentPoint(const World & world);

  RobotCommand RunFrame(const World & world, const Robot & robot);

  bool IsDone() const
  {
    return state_ == State::Done;
  }

  bool IsReady() const override
  {
    return state_ == State::FaceBall || state_ == State::KickBall;
  }

  double move_to_ball_velocity = 2.0;
  double robot_perp_dist_to_ball_threshold = 0.015;
  double angle_threshold = 0.1;
  double kick_drive_velocity = 0.4;

  // Allow the robot to push up against obstacles / other robots
  const bool cowabunga = true;

/*
                                          #@%%%%%%%%%%%%%=
                                  *%%%%%%%@@@@@@@@@@@@@@@@%%%%%%#
                             :%%%%%@@@@@@@%%%%%%@%%%%%%%%%@@@@@@@%%%%:
                          %%%%@@@@%%%%%%%%%::--------%%%%%%%%%%%%@@@@%%%%
                       %%%@@@@%%%%%+---:#%##%:----------@%%%%%%%%%%%%@@@@%%%
                    %%%@@@%%%%@:------------:@##%----------@%%%%%%%%%%%%@@@@%%%
                  %%@@@@%%%+-------##+++++*%*---@##:---------:%%%%%%%%%%%%%@@@%%%
                %%@@@@%%*-------+*%@##*   =%#**%--:%#@---------:@%%%@%%@%%%%%@@@%%%
              %%@@@%%@#-------+*%@#%  #+**#   +*%----*%%%*--------%@%%%%%%%%%%%@@@%%@
            #%@@@%%%--------#*%%@#  #*   %+**  +*@------:@%#%#%@%*=+@@@@@%%@@%%%%%@@%%=
          .%%@@%%%--------%*#%%@%#  **%%%%%+*. ****-------------::.::-@%%%@%%%@%%@@%@@%%
         %%%@@%%@%#**---==:-+%%@%#  +*@%%%@**  %%**:----------------------:%%%%@%@%@@@@%%:
        %%@@%%%#*-------------@@%%:  ******+   #%**+-------------------------*@@%%@%%@@@@%%
       #%@@%%@%@@@@#**#---------:@#@         -%%%@+**%------------%+****+%=----%@%%%%%%@@@%@
      %%@@%%@%#**#%@##@***=----------:-####+:-------****%-----=**#@=      @#*%--%@@%@@@%@@@%@
     #%@@%%*###########@%%#*+%----------------*#%#------:#@%%%*@@%  .+*#*** %%#--%%@%@@@@@@@%%
    %%@@%@*#:----------@*#*@##@%*************#@@@%*+-----------*@% @*:  %@*+ @*+-@@@@@%%%@@@@%*
   @%@@%%@:----------------:@##*@@@@@@%#@+#@%%%%%#%@%******-----%* +*@%%%%** %#%-:@@@@@%%%@@@%%
   %%@%%@--------------------------:---------------=@%%%%%%@#@------**@@@*** %##--@@@@%@%@@@@@%%
  %%@@@%---------------------------------------------------:@@%%#@=----##%  %@*---@@@@%%@@%@@@%%=
  %%@@@#------------------------------------------------------=------:%----+@@%---@@@@@@%%@@@@@%%
 %%@@%%-------------------------=%%*#################%#---=---------------%--%%---@@@@@@@%%%%@@%%
 %%@@%@---------------=@%##################################@==---------------@*---@@@@@@@@%%%@@@%#
 %@@%%+---------+%##############################################*--------------:--@@@@@@@%%%%%@@%%
%%@@%@*.::--#*#@@@@@%%%%%%%%%%%%%%%%%%%%%%%@@@@####################@---=---------#@@@%@%%%%@@@@@%%
@%@@%%      %@@@%%%@#. %%.       @@          @@%@%%%@@################%-----------=@@@@%@@@@@@@@%%:
%%@@@* *##% ###@---%     %%%%%%%%%%%%@+     .%        =@%%@##############@----------@@@@%@@@@@@@%%+
%%@@%%  ## %#####*+---#   %       %%%%%%%%%%%%:          .@=%%@#############---------@%@@@@@@%@@%%
@%@@%@##############*=----@       @%      @%%%%%%%%@@.  =@.    @%%%###########==-----=@@@%%%@@@@%%
-%@@@%#     ############%----=    %         %%     %%%%%%%%@      @%@###########------%@%%@@@@@@%%
 %%@@@ *### +##############%-----*@         %          %%%%%%%%%@ @@ %%@#########@----=@@@%@@@@@%%
 %%@@%#     *##################@-----=     @@          %     .@%%%%%   +%@#########----=@@%%%@@@%+
 @%@@@%##*#*.######################@-----:#@          @@        @%%@@%@   @%########--=-@@@%%@@%%
  %%@@@@#%  =###########################@-----=--:-%  %         %%  *%@%%%  %%######%---@%%@%@@%%
  %%@@%%       ###############################%@=--------:=+   %%      %%%%@  %@######--%@@@@@%%
   %%@@@@%@  +#########################################@#---=-=-%      %@  %%@ @@%####--@@@@@@%%
    %%@@%-      #############################################@---=--# @@    .@@%%%@###+-@@%@@%@
    %%%@%%%%%%%@%##@#############################################%=-----+   #@  %%%@##%%@@@@%%
     %%@@@%%       @#%@###############################################*--=-=-=%*%=-@%@%@@@@%%=
      %%@@%%@@ @  @%%%@@%%@################################################%%#######%%@@@@%@-
       %%%@%%%  -@%%  %%%%@@#%%@%#################################################%%@@@@@%%..
        %%%@@@%@%%   %  @@%%@@@@@%%%%%@##########################################%%@@@@@%%...
         :%%@@@%@  @    @%%%@@%%%@@@@@%%%%%%%@@##########################   ###%%@@@@@@%@::..
           %%@@@@%:   @@%  %%%%@@%%%%@@@@@@@@@@%%%%%%@%#################      @@@@@@@@@*:::.
            =%%@@@@%@%%:  @%   %@@@@%%%%%@@%%@@%@@@@@@@@@%#%%%%%%%%%  @%%%%@# %@@@@%%@-:::..
              %%%@@@%%%  %%  #@%  @%%%%%%%@%%@%%%%%@%@@@@@@@@@@- @@@@  +@@@=@@@@@@%@---:::.
                %%%@@@@@@   @@%   @  @@@@@@@%@%@%%@@@%@%@#@    %@@@@@%:  @@@@@@@@%+----::.
                  %%%@@@@@@%%%  @   :@%  @#  %@%   @@@@@  @@@%  @@@%@@@@@%@@@%%%==----:.
                    +%%%@@@@@@@@@-  @@  @%  *%@  @ %@@%@% .@@@@  @@@@@@@@@%@%@===---:.
                     .:#%%%@@@@@@@@@@%%     @%%     @@@@@  @@@@@@@@@@@@@%@%+====--:
                      .:::*%%%@@@@@@@@@@@@@@@@%#@@@@%@@@@@@@@@@@@@@@%%%%++++==-.
                       ..:::--#%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%%%##**++=.
                         ..::---===*%@%%%%@@@@@@@@@@%@@%%@%%%%%%%##*+:
                            .::---====+++**###%%%%%%%%%%%%###=
                                  .:--==+++++++=-.
*/

  void setPreKickOffset(double val)
  {
    pre_kick_offset = val;
  }

private:
  const double kDefaultPreKickOffset = kRobotRadius + kBallRadius + 0.06;
  double pre_kick_offset = kDefaultPreKickOffset;
  ateam_geometry::Point target_point_;

  enum class State
  {
    MoveBehindBall,
    FaceBall,
    KickBall,
    Done
  };
  State state_ = State::MoveBehindBall;

  ateam_geometry::Point GetPreKickPosition(const World & world);

  void ChooseState(const World & world, const Robot & robot);

  bool IsRobotBehindBall(const World & world, const Robot & robot, double hysteresis);
  bool IsRobotSettled(const World & world, const Robot & robot);
  bool IsRobotFacingBall(const Robot & robot);
  bool IsBallMoving(const World & world);

  RobotCommand RunMoveBehindBall(const World & world, const Robot & robot);
  RobotCommand RunFaceBall(const World & world, const Robot & robot);
  RobotCommand RunKickBall(const World & world, const Robot & robot);
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__LINE_KICK_HPP_
