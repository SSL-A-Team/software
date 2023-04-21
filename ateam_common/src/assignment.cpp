#include "ateam_common/assignment.hpp"

#include <unordered_set>

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
        marks(i, j) = 1;
        row_covers(i) = 1;
        col_covers(j) = 1;
        break;
      }
    }
  }

  return marks;
}

Eigen::MatrixXi ApplyStep4(Eigen::MatrixXi mark_matrix, const Eigen::MatrixXd & cost_matrix)
{
  Eigen::VectorXi row_covers = Eigen::VectorXi::Zero(matrix.rows());
  Eigen::VectorXi col_covers = Eigen::VectorXi::Zero(matrix.cols());

start:
  // Cover all cols containing a marked 0
  Eigen::VectorXi marked_col_coeff = mark_matrix.colwise().maxCoeff();
  for (int j = 0; j < mark_matrix.cols(); j++) {
    bool is_marked_zero = min_col_coeff(j) == 1;
    if (is_marked_zero) {
      col_covers(i) = 1;
    }
  }

  // Find any non-covered 0, and "prime" it
  for (int i = 0; i < cost_matrix.rows(); i++) {
    for (int j = 0; j < cost_matrix.cols(); j++) {
      bool is_covered_col = col_covers(i) == 1;
      bool is_zero = cost_col_coeff(i) == 0;
      if (!is_covered_col && is_zero) {
        mark_matrix(i, j) = 2;

        // If the zero is on the same row as a marked 0
        // Cover the corresponding row, and uncover the column of the
        // marked 0
        for (int jj = 0; jj < cost_matrix.cols(); jj++) {
          bool is_covered_zero = mark_matrix(i, jj) == 1;
          if (is_covered_zero) {
            row_covers(i) = 1;
            col_covers(jj) = 0;
            goto start;
          } else {
            // 2 step part
            // Find a zero in the col
            int iii = -1;
            for (int ii = 0; ii < cost_matrix.rows(); ii++) {
              bool is_covered_zero = mark_matrix(ii, j) == 1;
              if (is_covered_zero) {
                iii = i;
              }
            }
            if (iii != -1) {
              // find primed zero on corrisponding row
              // goto step one
            }
          }
        }
      }
    }
  }

}

}
}