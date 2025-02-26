// Copyright 2025 A Team
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

#include "cost_matrix_utilities.hpp"
#include <algorithm>

namespace ateam_common::assignment
{

Eigen::MatrixXd ScaleCostMatrix(
  const Eigen::MatrixXd & matrix, AssignmentType max_or_min)
{
  Eigen::MatrixXd new_matrix = matrix;
  if (max_or_min == AssignmentType::MinCost) {
    new_matrix *= -1;
  }
  // If any of the coefficients are NaN, ensure we ignore them using
  // Eigen's NaNPropagation template:
  // https://eigen.tuxfamily.org/index.php?title=3.4#Improvement_to_NaN_propagation
  double matMin = new_matrix.template minCoeff<Eigen::PropagateNumbers>();

  // If there are no negative values, no rescaling needed.
  if (matMin >= 0.0) {
    return new_matrix;
  }
  // Otherwise, shift so the entire matrix is moved to be > 0
  new_matrix -= (Eigen::MatrixXd::Ones(
      new_matrix.rows(), new_matrix.cols()) * matMin);
  return new_matrix;
}

Eigen::MatrixXd MakeSquareCostMatrix(
  const Eigen::MatrixXd & matrix)
{
  if (matrix.rows() == matrix.cols()) {
    return matrix;
  }

  std::size_t new_dim = std::max(matrix.rows(), matrix.cols());

  //  The value of any non-existant cost is 0, less than any assigable
  //  real cost.
  Eigen::MatrixXd new_mat =
    Eigen::MatrixXd::Constant(new_dim, new_dim, 0);

  new_mat.block(0, 0, matrix.rows(), matrix.cols()) = matrix;

  return new_mat;
}

Eigen::MatrixXd ReplaceNanCostsWithZeros(
  const Eigen::MatrixXd & matrix)
{
  Eigen::MatrixXd zero_mask = Eigen::MatrixXd::Zero(matrix.rows(), matrix.cols());
  Eigen::MatrixXd new_matrix = (matrix.array().isNaN()).select(zero_mask, matrix);
  return new_matrix;
}

Eigen::MatrixXd ReplaceNanCostsWithValue(
  const Eigen::MatrixXd & matrix,
  double value)
{
  Eigen::MatrixXd value_mask = Eigen::MatrixXd::Ones(matrix.rows(), matrix.cols()) * value;
  Eigen::MatrixXd new_matrix = (matrix.array().isNaN()).select(value_mask, matrix);
  return new_matrix;
}

Eigen::MatrixXd ReplaceForbiddenCostsWithZeros(
  const Eigen::MatrixXd & matrix,
  std::map<int, std::vector<int>> forbidden_assignments)
{
  Eigen::MatrixXd new_matrix = matrix;
  for (auto const & [row, col_vector] : forbidden_assignments) {
    for (auto col_ind : col_vector) {
      new_matrix(row, col_ind) = 0;
    }
  }
  return new_matrix;
}

Eigen::MatrixXd ReplaceForbiddenCostsWithValue(
  const Eigen::MatrixXd & matrix,
  std::map<int, std::vector<int>> forbidden_assignments,
  double value)
{
  Eigen::MatrixXd new_matrix = matrix;
  for (auto const & [row, col_vector] : forbidden_assignments) {
    for (auto col_ind : col_vector) {
      new_matrix(row, col_ind) = value;
    }
  }
  return new_matrix;
}

}  // namespace ateam_common::assignment
