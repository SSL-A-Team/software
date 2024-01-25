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

#include "ateam_common/assignment.hpp"

#include <unordered_set>
#include <vector>

#include <iostream>

namespace ateam_common::assignment
{
std::unordered_map<std::size_t, std::size_t> optimize_assignment(
  const Eigen::MatrixXd & cost_matrix)
{
  // cost matrix given as robots then goals(cols) coming in
  if (cost_matrix.rows() > cost_matrix.cols()) {
    auto transpose_ret = internal::optimize_assignment_impl(cost_matrix.transpose());
    std::unordered_map<std::size_t, std::size_t> ret;
    for (const auto & [key, value] : transpose_ret) {
      ret[value] = key;
    }
    return ret;
  } else {
    return internal::optimize_assignment_impl(cost_matrix);
  }
}

std::unordered_map<std::size_t, std::size_t> internal::optimize_assignment_impl(
  const Eigen::MatrixXd & cost_matrix)
{
  ::HungarianAlgorithm hungarian{};
  std::vector<int> result_vec;
  hungarian.Solve(cost_matrix, result_vec);

  std::unordered_map<std::size_t, std::size_t> result;
  for (size_t i = 0; i < result_vec.size(); i++) {
    result.emplace(i, result_vec[i]);
  }
  return result;

  // return ateam_common::assignment::hungarian<double>(cost_matrix);
  // return ateam_common::assignment::johnsons_hungarian<double>(cost_matrix);
  // return ateam_common::assignment::internal::optimize_assignment_impl_hungarian(<double>(cost_matrix);
}

std::unordered_map<std::size_t, std::size_t> internal::optimize_assignment_impl_hungarian(const Eigen::MatrixXd & cost_matrix)
{
  CostMarkCovers cmc;
  cmc.cost_matrix = squarize_matrix(cost_matrix);

  apply_step_1(cmc);
  apply_step_2(cmc);
  apply_step_3(cmc);

  uint32_t loops = 0;
  // TODO(COLLIN) Make this an actual const
  const uint32_t loop_limit = 40;
  do {
    apply_step_4(cmc);
    apply_step_5(cmc);
    loops++;
  } while (!has_unique_assignments(cmc) && loops < loop_limit);

  if (loops >= loop_limit) {
      // RCLCPP_ERROR_STREAM(rclcpp::get_logger(), "assignment ran for more than: " << std::to_string(loop_limit) << " loops");
      std::cout << "cost: \n" << cmc.cost_matrix << std::endl;
      std::cout << "assignment ran for more than: " << std::to_string(loop_limit) << " loops" <<std::endl;
  }
  // std::cout << "assignment ran for: " << std::to_string(loops) << " loops" <<std::endl;

  std::unordered_map<std::size_t, std::size_t> assignment;
  for (int i = 0; i < cmc.mark_matrix.rows(); i++) {
    bool is_row_inbounds = i < cost_matrix.rows();
    if (!is_row_inbounds) {
      continue;
    }

    for (int j = 0; j < cmc.mark_matrix.cols(); j++) {
      bool is_col_inbounds = j < cost_matrix.cols();
      if (!is_col_inbounds) {
        continue;
      }

      bool is_starred = cmc.mark_matrix(i, j) == ZerosType::STARRED;
      if (is_starred) {
        assignment[i] = j;
      }
    }
  }
  return assignment;
}

namespace internal
{
Eigen::MatrixXd squarize_matrix(const Eigen::MatrixXd & matrix)
{
  if (matrix.rows() == matrix.cols()) {
    return matrix;
  }

  std::size_t new_row_col = std::max(matrix.rows(), matrix.cols());
  Eigen::MatrixXd new_mat =
    Eigen::MatrixXd::Constant(new_row_col, new_row_col, INF);

  new_mat.block(0, 0, matrix.rows(), matrix.cols()) = matrix;

  return new_mat;
}

bool has_unique_assignments(const CostMarkCovers & cmc)
{
  // The count of marked rows and columns should be the size of the matrix
  int num_marked = 0;
  for (int i = 0; i < cmc.cost_matrix.rows(); i++) {
    num_marked += cmc.row_covers(i);
    num_marked += cmc.col_covers(i);
  }

  return num_marked == cmc.cost_matrix.rows();
}

void apply_step_1(CostMarkCovers & cmc)
{
  Eigen::VectorXd min_coeff = cmc.cost_matrix.rowwise().minCoeff();
  for (int i = 0; i < cmc.cost_matrix.rows(); i++) {
    cmc.cost_matrix.row(i) -= Eigen::MatrixXd::Constant(1, cmc.cost_matrix.cols(), min_coeff(i));
  }
}

void apply_step_2(CostMarkCovers & cmc)
{
  Eigen::VectorXd min_coeff = cmc.cost_matrix.colwise().minCoeff();
  for (int i = 0; i < cmc.cost_matrix.cols(); i++) {
    cmc.cost_matrix.col(i) -= Eigen::MatrixXd::Constant(cmc.cost_matrix.rows(), 1, min_coeff(i));
  }
}

void apply_step_3(CostMarkCovers & cmc)
{
  cmc.row_covers = Eigen::VectorXi::Zero(cmc.cost_matrix.rows());
  cmc.col_covers = Eigen::VectorXi::Zero(cmc.cost_matrix.cols());
  cmc.mark_matrix = Eigen::MatrixXi::Zero(cmc.cost_matrix.rows(), cmc.cost_matrix.cols());

  // Mark uncovered zeros one at a time
  for (int i = 0; i < cmc.cost_matrix.rows(); i++) {
    bool is_row_covered = cmc.row_covers(i) > 0;
    if (is_row_covered) {
      continue;
    }
    for (int j = 0; j < cmc.cost_matrix.cols(); j++) {
      // Can't count zeros that are already covered
      bool is_col_covered = cmc.col_covers(j) > 0;
      if (is_col_covered) {
        continue;
      }

      bool is_zero = cmc.cost_matrix(i, j) == 0;
      if (is_zero) {
        cmc.mark_matrix(i, j) = ZerosType::STARRED;
        cmc.row_covers(i) = 1;
        cmc.col_covers(j) = 1;
        break;
      }
    }
  }
}

void star_zero_cols(CostMarkCovers & cmc)
{
  cmc.col_covers = Eigen::VectorXi::Zero(cmc.mark_matrix.cols());
  Eigen::VectorXi marked_col_coeff = cmc.mark_matrix.colwise().maxCoeff();
  for (int j = 0; j < cmc.mark_matrix.cols(); j++) {
    bool is_starred_zero = marked_col_coeff(j) == ZerosType::STARRED;
    if (is_starred_zero) {
      cmc.col_covers(j) = 1;
    }
  }
}

std::optional<Eigen::Vector2i> next_uncovered_zero(
  const CostMarkCovers & cmc)
{
  for (int i = 0; i < cmc.cost_matrix.rows(); i++) {
    bool is_covered_row = cmc.row_covers(i) == 1;
    if (is_covered_row) {
      continue;
    }
    for (int j = 0; j < cmc.cost_matrix.cols(); j++) {
      bool is_covered_col = cmc.col_covers(j) == 1;
      if (is_covered_col) {
        continue;
      }

      bool is_zero = cmc.cost_matrix(i, j) == 0;
      if (is_zero) {
        return Eigen::Vector2i{i, j};
      }
    }
  }
  return std::nullopt;
}

std::optional<Eigen::Vector2i> find_zero_type_in_row(
  const CostMarkCovers & cmc,
  const Eigen::Vector2i & start,
  const ZerosType & target_type)
{
  for (int j = 0; j < cmc.mark_matrix.cols(); j++) {
    if (cmc.mark_matrix(start.x(), j) == target_type) {
      return Eigen::Vector2i{start.x(), j};
    }
  }
  return std::nullopt;
}

std::optional<Eigen::Vector2i> find_zero_type_in_col(
  const CostMarkCovers & cmc,
  const Eigen::Vector2i & start,
  const ZerosType & target_type)
{
  for (int i = 0; i < cmc.mark_matrix.cols(); i++) {
    if (cmc.mark_matrix(i, start.y()) == target_type) {
      return Eigen::Vector2i{i, start.y()};
    }
  }
  return std::nullopt;
}

void apply_step_4(CostMarkCovers & cmc)
{
  // Cover all columns containing a (starred) zero.
  cmc.row_covers = Eigen::VectorXi::Zero(cmc.cost_matrix.rows());
  cmc.col_covers = Eigen::VectorXi::Zero(cmc.cost_matrix.cols());

  // Cover all cols containing a starred 0
  star_zero_cols(cmc);

  while (true) {
    // Find a non-covered zero and prime it. (If all zeroes are covered, skip to step 5.)
    auto uncovered_zero = next_uncovered_zero(cmc);
    if (!uncovered_zero.has_value()) {
      return;
    }
    cmc.mark_matrix(uncovered_zero.value().x(), uncovered_zero.value().y()) = ZerosType::PRIMED;

    // If the zero is on the same row as a starred zero,
    // cover the corresponding row, and uncover the column of the starred zero.
    // Then, GOTO "Find a non-covered zero and prime it."
    auto starred_in_row = find_zero_type_in_row(
      cmc,
      uncovered_zero.value(),
      ZerosType::STARRED);
    if (starred_in_row.has_value()) {
      cmc.row_covers(uncovered_zero.value().x()) = 1;
      cmc.col_covers(starred_in_row.value().y()) = 0;
      continue;
    }

    // Else the non-covered zero has no assigned zero on its row.
    // We make a path starting from the zero by performing the following steps:
    std::vector<Eigen::Vector2i> path{uncovered_zero.value()};
    while (true) {
      // Substep 1: Find a starred zero on the corresponding column.
      // If there is one, go to Substep 2, else, stop.
      auto starred_in_row = find_zero_type_in_col(cmc, path.back(), ZerosType::STARRED);
      if (!starred_in_row.has_value()) {
        break;
      }
      path.push_back(starred_in_row.value());

      // Substep 2: Find a primed zero on the corresponding row (there should always be one).
      // Go to Substep 1.
      auto prime_in_row = find_zero_type_in_row(cmc, path.back(), ZerosType::PRIMED);
      path.push_back(prime_in_row.value());
    }

    // For all zeros encountered during the path, star primed zeros and unstar starred zeros.
    for (std::size_t k = 0; k < path.size(); k++) {
      if (cmc.mark_matrix(path.at(k).x(), path.at(k).y()) == ZerosType::PRIMED) {
        cmc.mark_matrix(path.at(k).x(), path.at(k).y()) = ZerosType::STARRED;
      } else if (cmc.mark_matrix(path.at(k).x(), path.at(k).y()) == ZerosType::STARRED) {
        cmc.mark_matrix(path.at(k).x(), path.at(k).y()) = ZerosType::NONE;
      }
    }

    // Unprime all primed zeroes and uncover all lines.
    for (int i = 0; i < cmc.mark_matrix.rows(); i++) {
      for (int j = 0; j < cmc.mark_matrix.cols(); j++) {
        if (cmc.mark_matrix(i, j) == ZerosType::PRIMED) {
          cmc.mark_matrix(i, j) = ZerosType::NONE;
        }
      }
    }
    cmc.row_covers = Eigen::VectorXi::Zero(cmc.cost_matrix.rows());
    cmc.col_covers = Eigen::VectorXi::Zero(cmc.cost_matrix.cols());

    // Cover all columns containing a (starred) zero.
    star_zero_cols(cmc);
  }
}

void apply_step_5(CostMarkCovers & cmc)
{
  // Find the lowest uncovered value
  double min_value = INF;
  for (int i = 0; i < cmc.cost_matrix.rows(); i++) {
    for (int j = 0; j < cmc.cost_matrix.cols(); j++) {
      bool is_row_covered = cmc.row_covers(i) == 1;
      bool is_col_covered = cmc.col_covers(j) == 1;
      if (!is_row_covered && !is_col_covered && cmc.cost_matrix(i, j) < min_value) {
        min_value = cmc.cost_matrix(i, j);
      }
    }
  }

  // Subtract this from every unmarked element and add it to every element covered by two lines
  for (int i = 0; i < cmc.cost_matrix.rows(); i++) {
    for (int j = 0; j < cmc.cost_matrix.cols(); j++) {
      bool is_row_covered = cmc.row_covers(i) == 1;
      bool is_col_covered = cmc.col_covers(j) == 1;
      if (!is_row_covered && !is_col_covered) {
        cmc.cost_matrix(i, j) -= min_value;
      } else if (is_row_covered && is_col_covered) {
        cmc.cost_matrix(i, j) += min_value;
      }
    }
  }
}

}  // namespace internal
}  // namespace ateam_common::assignment
