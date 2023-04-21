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

#ifndef ATEAM_COMMON__HUNGARIAN_HPP_
#define ATEAM_COMMON__HUNGARIAN_HPP_

#include <unordered_map>
#include <optional>

#include <Eigen/Dense>

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
  std::unordered_map<std::size_t, std::size_t> optimize_assignment(const Eigen::MatrixXd& cost_matrix);

namespace internal
{
  // Unfilled data when squarizing the matrix will be set to this value
  constexpr double UNFILLED_LARGE_VALUE = 1e10;

  enum ZerosType {
    NONE = 0,
    STARRED = 1,
    PRIMED = 2
  };

  /**
   * Given a non-square matrix, make the matrix square by adding the least number of rows/cols needed.
   * All new cells will be filled with some "large" cost
  */
  Eigen::MatrixXd SquarizeMatrix(const Eigen::MatrixXd & matrix);

  /**
   * Confirms that each row AND column have only 1 marked cell
  */
  bool HasUniqueAssignments(const Eigen::MatrixXi & mark_matrix);

  /**
   * For each row, the min element is subtracted from every element in that row
  */
  Eigen::MatrixXd ApplyStep1(const Eigen::MatrixXd & matrix);

  /**
   * For each col, the min element is subtracted from every element in that row
  */
  Eigen::MatrixXd ApplyStep2(const Eigen::MatrixXd & matrix);

  /**
   * Returns the mark matrix, each mark will be unique for that col/row
   *  * Only zeros are marked
   *  * From top left to bottom right, row-wise, each zero will be marked
   *  * Only 1 mark in each row and column is allowed
  */
  Eigen::MatrixXi ApplyStep3(const Eigen::MatrixXd & matrix);

  /**
   * Returns a mask vector corresponding to columns with a starred zero in the mark matrix
   */
  Eigen::VectorXi star_zero_cols(const Eigen::MatrixXi & mark_matrix);

  /**
   * Returns the matrix coordinates of the next uncovered zero
   * If none is found, return -1, -1
  */
  std::optional<Eigen::Vector2i> next_uncovered_zero(const Eigen::MatrixXd & cost_matrix, const Eigen::VectorXi & row_covers, const Eigen::VectorXi & col_covers);

  /**
   * Returns the matrix coordinates of the zero type requested in the same row as the start location
   * If none is found, return -1, -1
  */
  std::optional<Eigen::Vector2i> find_zero_type_in_row(
    const Eigen::MatrixXi & mark_matrix,
    const Eigen::Vector2i & start,
    const ZerosType & target_type);

  /**
   * Returns the matrix coordinates of the zero type requested in the same col as the start location
   * If none is found, return -1, -1
  */
  std::optional<Eigen::Vector2i> find_zero_type_in_col(
    const Eigen::MatrixXi & mark_matrix,
    const Eigen::Vector2i & start,
    const ZerosType & target_type);

  struct Covers {
    Eigen::VectorXi row_covers;
    Eigen::VectorXi col_covers;
  };

  /**
   * Using the mark matrix, optimize line coverage to as few lines as possible
  */
  Covers ApplyStep4(Eigen::MatrixXi mark_matrix, const Eigen::MatrixXd & cost_matrix);

  /**
   * Find the lowest uncovered value. Subtract this from every unmarked element and add it to every element covered by two lines.
  */
  Eigen::MatrixXd ApplyStep5(const Eigen::MatrixXi & mark_matrix, const Covers & covers);
}  // namespace internal
}  // namespace assignment
}  // namespace ateam_common

#endif  // ATEAM_COMMON__HUNGARIAN_HPP_
