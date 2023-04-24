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

bool HasUniqueAssignments(const Eigen::MatrixXi & mark_matrix)
{
  std::unordered_set<int> filled_rows(mark_matrix.rows());
  std::unordered_set<int> filled_cols(mark_matrix.cols());
  for (int i = 0; i < mark_matrix.cols(); i++) {
    for (int j = 0; j < mark_matrix.rows(); j++) {
      bool is_marked = mark_matrix(i, j) > 0;
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

  return static_cast<int>(filled_rows.size()) == mark_matrix.rows() && \
    static_cast<int>(filled_cols.size()) == mark_matrix.cols();
}

Eigen::MatrixXd ApplyStep1(const Eigen::MatrixXd & matrix)
{
  Eigen::MatrixXd new_mat = matrix;
  Eigen::VectorXd min_coeff = matrix.rowwise().minCoeff();
  for (int i = 0; i < matrix.rows(); i++) {
    new_mat.row(i) -= Eigen::MatrixXd::Constant(1, matrix.cols(), min_coeff(i));
  }
  return new_mat;
}

Eigen::MatrixXd ApplyStep2(const Eigen::MatrixXd & matrix)
{
  Eigen::MatrixXd new_mat = matrix;
  Eigen::VectorXd min_coeff = matrix.colwise().minCoeff();
  for (int i = 0; i < matrix.cols(); i++) {
    new_mat.col(i) -= Eigen::MatrixXd::Constant(matrix.rows(), 1, min_coeff(i));
  }
  return new_mat;
}

Eigen::MatrixXi ApplyStep3(const Eigen::MatrixXd & matrix)
{
  Eigen::VectorXi row_covers = Eigen::VectorXi::Zero(matrix.rows());
  Eigen::VectorXi col_covers = Eigen::VectorXi::Zero(matrix.cols());
  Eigen::MatrixXi marks = Eigen::MatrixXi::Zero(matrix.rows(), matrix.cols());

  // Mark uncovered zeros one at a time, 
  for (int i = 0; i < matrix.rows(); i++) {
    bool is_row_covered = row_covers(i) > 0;
    if (is_row_covered) {
      continue;
    }
    for (int j = 0; j < matrix.cols(); j++) {
      // Can't count zeros that are already covered
      bool is_col_covered = col_covers(j) > 0;
      if (is_col_covered) {
        continue;
      }

      bool is_zero = matrix(i, j) == 0;
      if (is_zero) {
        marks(i, j) = ZerosType::STARRED;
        row_covers(i) = 1;
        col_covers(j) = 1;
        break;
      }
    }
  }

  return marks;
}

Eigen::VectorXi star_zero_cols(const Eigen::MatrixXi & mark_matrix)
{
  Eigen::VectorXi col_covers = Eigen::VectorXi::Zero(mark_matrix.cols());
  Eigen::VectorXi marked_col_coeff = mark_matrix.colwise().maxCoeff();
  for (int j = 0; j < mark_matrix.cols(); j++) {
    bool is_starred_zero = marked_col_coeff(j) == ZerosType::STARRED;
    if (is_starred_zero) {
      col_covers(j) = 1;
    }
  }

  return col_covers;
}

std::optional<Eigen::Vector2i> next_uncovered_zero(
  const Eigen::MatrixXd & cost_matrix,
  const Eigen::VectorXi & row_covers,
  const Eigen::VectorXi & col_covers)
{
  for (int i = 0; i < cost_matrix.rows(); i++) {
    for (int j = 0; j < cost_matrix.cols(); j++) {
      bool is_covered_col = col_covers(j) == 1;
      bool is_covered_row = row_covers(i) == 1;
      bool is_zero = cost_matrix(i, j) == 0;
      if (!is_covered_col && !is_covered_row && is_zero) {
        return Eigen::Vector2i{i, j};
      }
    }
  }
  return std::nullopt;
}

std::optional<Eigen::Vector2i> find_zero_type_in_row(
  const Eigen::MatrixXi & mark_matrix,
  const Eigen::Vector2i & start,
  const ZerosType & target_type)
{
  for (int j = 0; j < mark_matrix.cols(); j++) {
    if (mark_matrix(start.x(), j) == target_type) {
      return Eigen::Vector2i{start.x(), j};
    }
  }
  return std::nullopt;
}

std::optional<Eigen::Vector2i> find_zero_type_in_col(
  const Eigen::MatrixXi & mark_matrix,
  const Eigen::Vector2i & start,
  const ZerosType & target_type)
{
  for (int i = 0; i < mark_matrix.cols(); i++) {
    if (mark_matrix(i, start.y()) == target_type) {
      return Eigen::Vector2i{i, start.y()};
    }
  }
  return std::nullopt;
}

Covers ApplyStep4(Eigen::MatrixXi mark_matrix, const Eigen::MatrixXd & cost_matrix)
{
  // Cover all columns containing a (starred) zero.
  Eigen::VectorXi row_covers = Eigen::VectorXi::Zero(cost_matrix.rows());
  Eigen::VectorXi col_covers = Eigen::VectorXi::Zero(cost_matrix.cols());

  // Cover all cols containing a starred 0
  col_covers = star_zero_cols(mark_matrix);

  while (true) {
    // Find a non-covered zero and prime it. (If all zeroes are covered, skip to step 5.)
    auto uncovered_zero = next_uncovered_zero(cost_matrix, row_covers, col_covers);
    if (!uncovered_zero.has_value()) {
      return Covers{.row_covers=row_covers, col_covers=col_covers};
    }
    mark_matrix(uncovered_zero.value().x(), uncovered_zero.value().y()) = ZerosType::PRIMED;

    // If the zero is on the same row as a starred zero,
    // cover the corresponding row, and uncover the column of the starred zero.
    // Then, GOTO "Find a non-covered zero and prime it."
    auto starred_in_row = find_zero_type_in_row(
      mark_matrix,
      uncovered_zero.value(),
      ZerosType::STARRED);
    if (starred_in_row.has_value()) {
      row_covers(uncovered_zero.value().x()) = 1;
      col_covers(starred_in_row.value().y()) = 0;
      continue;
    }

    // Else the non-covered zero has no assigned zero on its row.
    // We make a path starting from the zero by performing the following steps:
    std::vector<Eigen::Vector2i> path{uncovered_zero.value()};
    while(true) {
      // Substep 1: Find a starred zero on the corresponding column.
      // If there is one, go to Substep 2, else, stop.
      auto starred_in_row = find_zero_type_in_col(mark_matrix, path.back(), ZerosType::STARRED);
      if (!starred_in_row.has_value()) {
        break;
      }
      path.push_back(starred_in_row.value());

      // Substep 2: Find a primed zero on the corresponding row (there should always be one).
      // Go to Substep 1.
      auto prime_in_row = find_zero_type_in_row(mark_matrix, path.back(), ZerosType::PRIMED);
      path.push_back(prime_in_row.value());
    }

    // For all zeros encountered during the path, star primed zeros and unstar starred zeros.
    for (std::size_t k = 0; k < path.size(); k++) {
      if (mark_matrix(path.at(k).x(), path.at(k).y()) == ZerosType::PRIMED) {
        mark_matrix(path.at(k).x(), path.at(k).y()) = ZerosType::STARRED;
      } else if (mark_matrix(path.at(k).x(), path.at(k).y()) == ZerosType::STARRED) {
        mark_matrix(path.at(k).x(), path.at(k).y()) = ZerosType::NONE;
      }
    }

    // Unprime all primed zeroes and uncover all lines.
    for (int i = 0; i < mark_matrix.rows(); i++) {
      for (int j = 0; j < mark_matrix.cols(); j++) {
        if (mark_matrix(i, j) == ZerosType::PRIMED) {
          mark_matrix(i, j) = ZerosType::NONE;
        }
      }
    }
    row_covers = Eigen::VectorXi::Zero(cost_matrix.rows());
    col_covers = Eigen::VectorXi::Zero(cost_matrix.cols());
    
    // Cover all columns containing a (starred) zero.
    col_covers = star_zero_cols(mark_matrix);
  }
}

Eigen::MatrixXd ApplyStep5(Eigen::MatrixXd cost_matrix, const Covers & covers)
{
  // Find the lowest uncovered value
  double min_value = UNFILLED_LARGE_VALUE;
  for (int i = 0; i < cost_matrix.rows(); i++) {
    for (int j = 0; j < cost_matrix.cols(); j++) {
      bool is_row_covered = covers.row_covers(i) == 1;
      bool is_col_covered = covers.col_covers(j) == 1;
      if (!is_row_covered && !is_col_covered && cost_matrix(i, j) < min_value) {
        min_value = cost_matrix(i, j);
      }
    }
  }

  // Subtract this from every unmarked element and add it to every element covered by two lines
  for (int i = 0; i < cost_matrix.rows(); i++) {
    for (int j = 0; j < cost_matrix.cols(); j++) {
      bool is_row_covered = covers.row_covers(i) == 1;
      bool is_col_covered = covers.col_covers(j) == 1;
      if (!is_row_covered && !is_col_covered) {
        cost_matrix(i, j) -= min_value;
      } else if (is_row_covered && is_col_covered) {
        cost_matrix(i, j) += min_value;
      }
    }
  }

  return cost_matrix;
}
}
}