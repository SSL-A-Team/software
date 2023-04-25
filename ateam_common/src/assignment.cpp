#include "ateam_common/assignment.hpp"

#include <unordered_set>
#include <vector>

#include <iostream>

namespace ateam_common::assignment
{
std::unordered_map<std::size_t, std::size_t> optimize_assignment(const Eigen::MatrixXd& cost_matrix)
{
  return {};
}

namespace internal
{
Eigen::MatrixXd SquarizeMatrix(const Eigen::MatrixXd& matrix)
{
  if (matrix.rows() == matrix.cols()) {
    return matrix;
  }

  std::size_t new_row_col = std::max(matrix.rows(), matrix.cols());
  Eigen::MatrixXd new_mat = Eigen::MatrixXd::Constant(new_row_col, new_row_col, UNFILLED_LARGE_VALUE);

  new_mat.block(0, 0, matrix.rows(), matrix.cols()) = matrix;

  return new_mat;
}

bool HasUniqueAssignments(const CostMarkCovers & cmc)
{
  std::unordered_set<int> filled_rows(cmc.mark_matrix.rows());
  std::unordered_set<int> filled_cols(cmc.mark_matrix.cols());
  for (int i = 0; i < cmc.mark_matrix.cols(); i++) {
    for (int j = 0; j < cmc.mark_matrix.rows(); j++) {
      bool is_marked = cmc.mark_matrix(i, j) > 0;
      if (is_marked) {

        // If already marked in the row / col, we know isn't invalid
        bool already_marked_row = filled_rows.count(i) > 0;
        bool already_marked_col = filled_cols.count(j) > 0;
        if (!already_marked_row && !already_marked_col) {
          filled_rows.insert(i);
          filled_cols.insert(j);
        } else {
          return false;
        }
      }
    }
  }

  return static_cast<int>(filled_rows.size()) == cmc.mark_matrix.rows() && \
    static_cast<int>(filled_cols.size()) == cmc.mark_matrix.cols();
}

void ApplyStep1(CostMarkCovers & cmc)
{
  Eigen::VectorXd min_coeff = cmc.cost_matrix.rowwise().minCoeff();
  for (int i = 0; i < cmc.cost_matrix.rows(); i++) {
    cmc.cost_matrix.row(i) -= Eigen::MatrixXd::Constant(1, cmc.cost_matrix.cols(), min_coeff(i));
  }
}

void ApplyStep2(CostMarkCovers & cmc)
{
  Eigen::VectorXd min_coeff = cmc.cost_matrix.colwise().minCoeff();
  for (int i = 0; i < cmc.cost_matrix.cols(); i++) {
    cmc.cost_matrix.col(i) -= Eigen::MatrixXd::Constant(cmc.cost_matrix.rows(), 1, min_coeff(i));
  }
}

void ApplyStep3(CostMarkCovers & cmc)
{
  cmc.row_covers = Eigen::VectorXi::Zero(cmc.cost_matrix.rows());
  cmc.col_covers = Eigen::VectorXi::Zero(cmc.cost_matrix.cols());
  cmc.mark_matrix = Eigen::MatrixXi::Zero(cmc.cost_matrix.rows(), cmc.cost_matrix.cols());

  // Mark uncovered zeros one at a time, 
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
    for (int j = 0; j < cmc.cost_matrix.cols(); j++) {
      bool is_covered_col = cmc.col_covers(j) == 1;
      bool is_covered_row = cmc.row_covers(i) == 1;
      bool is_zero = cmc.cost_matrix(i, j) == 0;
      if (!is_covered_col && !is_covered_row && is_zero) {
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

void ApplyStep4(CostMarkCovers & cmc)
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
    while(true) {
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

void ApplyStep5(CostMarkCovers & cmc)
{
  // Find the lowest uncovered value
  double min_value = UNFILLED_LARGE_VALUE;
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
}
}