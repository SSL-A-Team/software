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

#ifndef ATEAM_COMMON__ASSIGNMENT_HPP_
#define ATEAM_COMMON__ASSIGNMENT_HPP_

#include <Eigen/Dense>

#include <unordered_map>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <ateam_common/assignment_strategies.hpp>
#include <ateam_common/hungarian.hpp>

namespace ateam_common
{
namespace assignment
{

/**
 * Given a Matrix of N rows and M columns, where...
 *  * each of the N rows is a robot to assign
 *  * each of the M columns is a role to be assigned
 *  * each cell at (i,j) is the cost of robot i performing role j
 *
 * @return List of (i, j) mappings for the most optimial assignment
 *
 * @note Each role/robot will only be assigned at most once
*/
std::unordered_map<std::size_t, std::size_t> optimize_assignment(
  const Eigen::MatrixXd & cost_matrix);

namespace internal
{
/**
 * Implements actual algorithm steps in order
*/
std::unordered_map<std::size_t, std::size_t> optimize_assignment_impl(
  const Eigen::MatrixXd & cost_matrix);

std::unordered_map<std::size_t, std::size_t> optimize_assignment_impl_hungarian(
  const Eigen::MatrixXd & cost_matrix);


// Unfilled data when squarizing the matrix will be set to this value
constexpr double INF = 0;

// Marked zeros can have multiple types
enum ZerosType
{
  NONE = 0,
  STARRED = 1,
  PRIMED = 2
};

struct CostMarkCovers
{
  Eigen::MatrixXd cost_matrix;  // Valid at all times
  Eigen::MatrixXi mark_matrix;  // Valid after Step 3
  Eigen::VectorXi row_covers;  // Valid after Step 4
  Eigen::VectorXi col_covers;  // Valid after Step 4
};

/**
 * Given a non-square matrix, make the matrix square by adding the least number of rows/cols needed.
 * All new cells will be filled with some "large" cost
*/
Eigen::MatrixXd squarize_matrix(const Eigen::MatrixXd & matrix);

/**
 * @return True if there is an unique and optimial assignment
*/
bool has_unique_assignments(const CostMarkCovers & cmc);

/**
 * For each row in the cost matrix, the min element is subtracted from every element in that row
*/
void apply_step_1(CostMarkCovers & cmc);

/**
 * For each col in the cost matrix, the min element is subtracted from every element in that row
*/
void apply_step_2(CostMarkCovers & cmc);

/**
 * Creates the mark matrix, each mark will be unique for that col/row
 *  * Only zeros are marked
 *  * From top left to bottom right, row-wise, each zero will be marked
 *  * Only 1 mark in each row and column is allowed
*/
void apply_step_3(CostMarkCovers & cmc);

/**
 * Create the col covers for all starred zeros in the mark matrix
 */
void star_zero_cols(CostMarkCovers & cmc);

/**
 * Returns the matrix coordinates of the next uncovered zero
 * If none is found, return -1, -1
*/
std::optional<Eigen::Vector2i> next_uncovered_zero(const CostMarkCovers & cmc);

/**
 * Returns the matrix coordinates of the zero type requested in the same row as the start location
 * If none is found, return -1, -1
*/
std::optional<Eigen::Vector2i> find_zero_type_in_row(
  const CostMarkCovers & cmc,
  const Eigen::Vector2i & start,
  const ZerosType & target_type);

/**
 * Returns the matrix coordinates of the zero type requested in the same col as the start location
 * If none is found, return -1, -1
*/
std::optional<Eigen::Vector2i> find_zero_type_in_col(
  const CostMarkCovers & cmc,
  const Eigen::Vector2i & start,
  const ZerosType & target_type);

/**
 * Using the mark matrix, optimize line coverage to as few lines as possible.
 * Lines refer to the col/row covers
*/
void apply_step_4(CostMarkCovers & cmc);

/**
 * Find the lowest uncovered value. Subtract this from every unmarked element and add it to every element covered by two lines.
*/
void apply_step_5(CostMarkCovers & cmc);
}  // namespace internal
}  // namespace assignment
}  // namespace ateam_common

#endif  // ATEAM_COMMON__ASSIGNMENT_HPP_
