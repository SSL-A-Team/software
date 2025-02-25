// Copyright 2024 A Team
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


#ifndef PLAY_HELPERS__ROBOT_ASSIGNMENT_HPP_
#define PLAY_HELPERS__ROBOT_ASSIGNMENT_HPP_

#include <vector>
#include <optional>
#include <string>
#include <unordered_map>
#include "types/robot.hpp"

namespace ateam_kenobi::play_helpers
{

std::vector<std::optional<Robot>> assignRobots(
  const std::vector<Robot> & available_robots,
  const std::vector<ateam_geometry::Point> & positions,
  const std::vector<std::vector<int>> & disallowed_robot_ids = {});

struct GroupRecord
{
  std::string name;
  std::size_t start_index;
  std::size_t size;
};

class GroupAssignmentSet
{
public:
  void AddGroup(
    const std::string & name, const std::vector<ateam_geometry::Point> & points,
    const std::vector<std::vector<int>> & disallowed_ids = {});

  void AddPosition(
    const std::string & name, const ateam_geometry::Point & point,
    const std::vector<int> & disallowed_ids = {});

  std::vector<ateam_geometry::Point> GetPoints() const
  {
    return points_;
  }

  std::vector<std::vector<int>> GetDisallowedIds() const
  {
    return disallowed_ids_;
  }

  std::vector<GroupRecord> GetRecords() const
  {
    return records_;
  }

private:
  std::vector<GroupRecord> records_;
  std::vector<ateam_geometry::Point> points_;
  std::vector<std::vector<int>> disallowed_ids_;

  void AddDisallowedIds(const std::vector<int> & disallowed_ids);
  void AddDisallowedIds(
    const std::vector<std::vector<int>> & disallowed_ids,
    const std::size_t & num_new_points);
};

class GroupAssignmentResult
{
public:
  GroupAssignmentResult() = default;

  GroupAssignmentResult(
    std::vector<GroupRecord> records,
    std::vector<std::optional<Robot>> assignments);

  std::vector<std::optional<Robot>> GetGroupAssignments(const std::string & name) const;

  std::optional<Robot> GetPositionAssignment(const std::string & name) const;

  std::vector<Robot> GetGroupFilledAssignments(const std::string & name) const;

  /**
   * @brief Same as @c GetGroupFilledAssignments but if there is no group with the given name, this function returns an empty array instead of throwing.
   */
  std::vector<Robot> GetGroupFilledAssignmentsOrEmpty(const std::string & name) const;

  template<typename PositionFunc>
  void RunPositionIfAssigned(const std::string & name, PositionFunc func) const
  {
    const auto & maybe_robot = GetPositionAssignment(name);
    if (!maybe_robot) {
      return;
    }
    func(*maybe_robot);
  }

private:
  std::vector<GroupRecord> records_;
  std::vector<std::optional<Robot>> assignments_;

  const GroupRecord & GetRecord(const std::string & name) const;
};

GroupAssignmentResult assignGroups(
  const std::vector<Robot> & available_robots,
  const GroupAssignmentSet & groups);

}  // namespace ateam_kenobi::play_helpers

#endif  // PLAY_HELPERS__ROBOT_ASSIGNMENT_HPP_
