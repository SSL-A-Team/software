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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "ateam_common/assignment.hpp"

namespace assignment = ateam_common::assignment;
namespace ainternal = ateam_common::assignment::internal;

TEST(Assignment, optimize_assignment) {
  Eigen::Matrix<double, 3, 3> cost3x3{{8, 4, 7}, {5, 2, 3}, {9, 4, 8}};
  auto out3x3 = assignment::optimize_assignment(cost3x3);
  EXPECT_EQ(out3x3.size(), 3);
  EXPECT_EQ(out3x3.at(0), 0);
  EXPECT_EQ(out3x3.at(1), 2);
  EXPECT_EQ(out3x3.at(2), 1);

  Eigen::Matrix<double, 5, 5> cost5x5{
    {10, 5, 13, 15, 16},
    {3, 9, 18, 13, 6},
    {10, 7, 2, 2, 2},
    {7, 11, 9, 7, 12},
    {7, 9, 10, 4, 12}
  };
  auto out5x5 = assignment::optimize_assignment(cost5x5);
  EXPECT_EQ(out5x5.size(), 5);
  EXPECT_EQ(out5x5.at(0), 1);
  EXPECT_EQ(out5x5.at(1), 0);
  EXPECT_EQ(out5x5.at(2), 4);
  EXPECT_EQ(out5x5.at(3), 2);
  EXPECT_EQ(out5x5.at(4), 3);

  Eigen::Matrix<double, 3, 2> cost3x2{
    {1, 10},
    {10, 1},
    {5, 5}
  };
  auto out3x2 = assignment::optimize_assignment(cost3x2);
  EXPECT_EQ(out3x2.size(), 2);
  EXPECT_EQ(out3x2.at(0), 0);
  EXPECT_EQ(out3x2.at(1), 1);

  Eigen::Matrix<double, 2, 3> cost2x3{
    {1, 10, 5},
    {10, 1, 5},
  };
  auto out2x3 = assignment::optimize_assignment(cost2x3);
  EXPECT_EQ(out2x3.size(), 2);
  EXPECT_EQ(out2x3.at(0), 0);
  EXPECT_EQ(out2x3.at(1), 1);

  Eigen::Matrix<double, 8, 1> cost8x1{{3.1}, {3.09}, {3.64}, {3.44}, {3.47}, {3.44}, {3.85},
    {3.44}};
  auto out8x1 = assignment::optimize_assignment(cost8x1.transpose());
  EXPECT_EQ(out8x1.size(), 1);
  EXPECT_EQ(out8x1.at(0), 1);
}

MATCHER_P(IsSameSize, m, "") {
  *result_listener << "where the size (" << arg.rows() << ", " << arg.cols() << ") ";
  *result_listener << "!= expected (" << m.rows() << ", " << m.cols() << ")";
  return arg.rows() == m.rows() && arg.cols() == m.cols();
}

MATCHER_P(HasOriginalData, original, "") {
  // The original data should stay the same
  *result_listener << "where the new data:\n\r" << arg << "\n\r";
  *result_listener << "is not the original data:\n\r" << original << "\n\r";
  return (arg.block(
           0, 0, original.rows(),
           original.cols()).array() - original.array()).matrix().sum() < 1e-6;
}

MATCHER_P(HasUnfilledData, original, "") {
  *result_listener << "where the unfilled data:\n\r" << arg << "\n\r";
  *result_listener << "is incorrect compared to original data:\n\r" << original << "\n\r";
  if (arg.size() == original.size()) {
    return true;
  }

  // Average cell value in the non-original cells should be INF
  Eigen::MatrixXd temp = arg;
  std::size_t diff = arg.size() - original.size();
  temp.block(0, 0, original.rows(), original.cols()) = Eigen::MatrixXd::Zero(
    original.rows(), original.cols());
  return temp.sum() / diff == ainternal::INF;
}

TEST(Assignment, SquareizeMatrix)
{
  // No-op with square input
  Eigen::Matrix<double, 1, 1> square1x1{{1}};
  EXPECT_THAT(ainternal::squarize_matrix(square1x1), IsSameSize(square1x1));
  EXPECT_THAT(ainternal::squarize_matrix(square1x1), HasOriginalData(square1x1));
  EXPECT_THAT(ainternal::squarize_matrix(square1x1), HasUnfilledData(square1x1));

  Eigen::Matrix2d square2x2{{1, 2}, {3, 4}};
  EXPECT_THAT(ainternal::squarize_matrix(square2x2), IsSameSize(square2x2));
  EXPECT_THAT(ainternal::squarize_matrix(square2x2), HasOriginalData(square2x2));
  EXPECT_THAT(ainternal::squarize_matrix(square2x2), HasUnfilledData(square2x2));

  Eigen::Matrix3d square3x3{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  EXPECT_THAT(ainternal::squarize_matrix(square3x3), IsSameSize(square3x3));
  EXPECT_THAT(ainternal::squarize_matrix(square3x3), HasOriginalData(square3x3));
  EXPECT_THAT(ainternal::squarize_matrix(square3x3), HasUnfilledData(square3x3));

  // Too many rows
  Eigen::Matrix<double, 2, 1> row2x1{{1}, {2}};
  EXPECT_THAT(ainternal::squarize_matrix(row2x1), IsSameSize(square2x2));
  EXPECT_THAT(ainternal::squarize_matrix(row2x1), HasOriginalData(row2x1));
  EXPECT_THAT(ainternal::squarize_matrix(row2x1), HasUnfilledData(row2x1));

  Eigen::Matrix<double, 3, 1> row3x1{{1}, {2}, {3}};
  EXPECT_THAT(ainternal::squarize_matrix(row3x1), IsSameSize(square3x3));
  EXPECT_THAT(ainternal::squarize_matrix(row3x1), HasOriginalData(row3x1));
  EXPECT_THAT(ainternal::squarize_matrix(row3x1), HasUnfilledData(row3x1));

  // Too many cols
  Eigen::Matrix<double, 2, 1> col1x2{{1, 2}};
  EXPECT_THAT(ainternal::squarize_matrix(col1x2), IsSameSize(square2x2));
  EXPECT_THAT(ainternal::squarize_matrix(col1x2), HasOriginalData(col1x2));
  EXPECT_THAT(ainternal::squarize_matrix(col1x2), HasUnfilledData(col1x2));

  Eigen::Matrix<double, 3, 1> col1x3{{1, 2, 3}};
  EXPECT_THAT(ainternal::squarize_matrix(col1x3), IsSameSize(square3x3));
  EXPECT_THAT(ainternal::squarize_matrix(col1x3), HasOriginalData(col1x3));
  EXPECT_THAT(ainternal::squarize_matrix(col1x3), HasUnfilledData(col1x3));
}

MATCHER_P(Has0ReferencedRows, original, "") {
  *result_listener << "where the rows aren't relative to the min value:\n\r" << arg << "\n\r";
  *result_listener << "with the original matrix:\n\r" << original << "\n\r";
  bool valid = true;
  // For each row, All data in that row should be relative to the min value in the original matrix
  Eigen::VectorXd min_coeff = original.rowwise().minCoeff();
  for (int i = 0; i < arg.rows(); i++) {
    for (int j = 0; j < arg.cols(); j++) {
      if (original(i, j) - min_coeff(i) != arg(i, j)) {
        *result_listener << "at (row, col): " << i << " " << j << "\n\r";
        valid = false;
      }
    }
  }

  return valid;
}

TEST(Assignment, apply_step_1)
{
  ainternal::CostMarkCovers cmc1x1;
  cmc1x1.cost_matrix = Eigen::Matrix<double, 1, 1>{{1}};
  ainternal::apply_step_1(cmc1x1);
  EXPECT_THAT(cmc1x1.cost_matrix, Has0ReferencedRows(Eigen::Matrix<double, 1, 1>{{1}}));

  ainternal::CostMarkCovers cmc2x2;
  cmc2x2.cost_matrix = Eigen::Matrix2d{{1, 2}, {3, 4}};
  ainternal::apply_step_1(cmc2x2);
  EXPECT_THAT(cmc2x2.cost_matrix, Has0ReferencedRows(Eigen::Matrix2d{{1, 2}, {3, 4}}));

  ainternal::CostMarkCovers cmc3x3;
  cmc3x3.cost_matrix = Eigen::Matrix3d{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  ainternal::apply_step_1(cmc3x3);
  EXPECT_THAT(
    cmc3x3.cost_matrix,
    Has0ReferencedRows(Eigen::Matrix3d{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}));
}

MATCHER_P(Has0ReferencedCols, original, "") {
  *result_listener << "where the rows aren't relative to the min value:\n\r" << arg << "\n\r";
  *result_listener << "with the original matrix:\n\r" << original << "\n\r";
  bool valid = true;
  // For each col, All data in that col should be relative to the min value in the original matrix
  Eigen::VectorXd min_coeff = original.colwise().minCoeff();
  for (int j = 0; j < arg.cols(); j++) {
    for (int i = 0; i < arg.rows(); i++) {
      if (original(i, j) - min_coeff(j) != arg(i, j)) {
        *result_listener << "at (row, col): " << i << " " << j << "\n\r";
        valid = false;
      }
    }
  }

  return valid;
}

TEST(Assignment, apply_step_2)
{
  ainternal::CostMarkCovers cmc1x1;
  cmc1x1.cost_matrix = Eigen::Matrix<double, 1, 1>{{1}};
  ainternal::apply_step_2(cmc1x1);
  EXPECT_THAT(cmc1x1.cost_matrix, Has0ReferencedCols(Eigen::Matrix<double, 1, 1>{{1}}));

  ainternal::CostMarkCovers cmc2x2;
  cmc2x2.cost_matrix = Eigen::Matrix2d{{1, 2}, {3, 4}};
  ainternal::apply_step_2(cmc2x2);
  EXPECT_THAT(cmc2x2.cost_matrix, Has0ReferencedCols(Eigen::Matrix2d{{1, 2}, {3, 4}}));

  ainternal::CostMarkCovers cmc3x3;
  cmc3x3.cost_matrix = Eigen::Matrix3d{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  ainternal::apply_step_2(cmc3x3);
  EXPECT_THAT(
    cmc3x3.cost_matrix,
    Has0ReferencedCols(Eigen::Matrix3d{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}));
}

MATCHER_P(IsSame, original, "") {
  *result_listener << "where :\n\r" << arg << "\n\r";
  *result_listener << "is not equal to:\n\r" << original << "\n\r";
  bool valid = true;
  for (int j = 0; j < arg.cols(); j++) {
    for (int i = 0; i < arg.rows(); i++) {
      if (original(i, j) != arg(i, j)) {
        *result_listener << "at (row, col): " << i << " " << j << "\n\r";
        valid = false;
      }
    }
  }

  return valid;
}

TEST(Assignment, apply_step_3)
{
  // Straightforward marking
  ainternal::CostMarkCovers cmc1x1;
  cmc1x1.cost_matrix = Eigen::Matrix<double, 1, 1>{{0}};
  ainternal::apply_step_3(cmc1x1);
  EXPECT_THAT(cmc1x1.mark_matrix, IsSame(Eigen::Matrix<double, 1, 1>{{1}}));

  ainternal::CostMarkCovers cmc2x2;
  cmc2x2.cost_matrix = Eigen::Matrix2d{{0, 2}, {3, 0}};
  ainternal::apply_step_3(cmc2x2);
  EXPECT_THAT(cmc2x2.mark_matrix, IsSame(Eigen::Matrix2d{{1, 0}, {0, 1}}));

  ainternal::CostMarkCovers cmc3x3;
  cmc3x3.cost_matrix = Eigen::Matrix3d{{1, 0, 3}, {0, 5, 6}, {7, 8, 0}};
  ainternal::apply_step_3(cmc3x3);
  EXPECT_THAT(cmc3x3.mark_matrix, IsSame(Eigen::Matrix3d{{0, 1, 0}, {1, 0, 0}, {0, 0, 1}}));

  // More complicated marking
  ainternal::CostMarkCovers zero2x2;
  zero2x2.cost_matrix = Eigen::Matrix2d{{0, 0}, {0, 0}};
  ainternal::apply_step_3(zero2x2);
  EXPECT_THAT(zero2x2.mark_matrix, IsSame(Eigen::Matrix2d{{1, 0}, {0, 1}}));

  ainternal::CostMarkCovers inv2x2;
  inv2x2.cost_matrix = Eigen::Matrix2d{{1, 0}, {0, 1}};
  ainternal::apply_step_3(inv2x2);
  EXPECT_THAT(inv2x2.mark_matrix, IsSame(Eigen::Matrix2d{{0, 1}, {1, 0}}));
}

TEST(Assignment, apply_step_4)
{
  ainternal::CostMarkCovers cmc1x1;
  cmc1x1.cost_matrix = Eigen::Matrix<double, 1, 1>{{0}};
  cmc1x1.mark_matrix = Eigen::Matrix<int, 1, 1>{{0}};
  ainternal::apply_step_4(cmc1x1);
  EXPECT_THAT(cmc1x1.row_covers, IsSame(Eigen::Vector<int, 1>{0}));
  EXPECT_THAT(cmc1x1.col_covers, IsSame(Eigen::Vector<int, 1>{1}));

  ainternal::CostMarkCovers cmc4x4;
  cmc4x4.cost_matrix =
    Eigen::Matrix<double, 4, 4>{{0, 12, 0, 14}, {21, 0, 23, 0}, {0, 32, 33, 34}, {0, 42, 43, 44}};
  cmc4x4.mark_matrix =
    Eigen::Matrix<int, 4, 4>{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
  ainternal::apply_step_4(cmc4x4);
  EXPECT_THAT(cmc4x4.row_covers, IsSame(Eigen::Vector<int, 4>{0, 1, 0, 0}));
  EXPECT_THAT(cmc4x4.col_covers, IsSame(Eigen::Vector<int, 4>{1, 0, 1, 0}));
}

TEST(Assignment, apply_step_5)
{
  ainternal::CostMarkCovers cmc1x1;
  cmc1x1.cost_matrix = Eigen::Matrix<double, 1, 1>{{1}};
  cmc1x1.mark_matrix = Eigen::Matrix<int, 1, 1>{{0}};
  cmc1x1.row_covers = Eigen::Vector<int, 1>{0};
  cmc1x1.col_covers = Eigen::Vector<int, 1>{0};
  ainternal::apply_step_5(cmc1x1);
  EXPECT_THAT(cmc1x1.cost_matrix, IsSame(Eigen::Matrix<double, 1, 1>{{0}}));

  ainternal::CostMarkCovers cmc4x4;
  cmc4x4.cost_matrix =
    Eigen::Matrix<double, 4, 4>{{0, 12, 0, 14}, {21, 0, 23, 0}, {0, 32, 33, 34}, {0, 42, 43, 44}};
  cmc4x4.mark_matrix = Eigen::Matrix<int, 4, 4>::Zero();
  cmc4x4.row_covers = Eigen::Vector<int, 4>{0, 1, 0, 0};
  cmc4x4.col_covers = Eigen::Vector<int, 4>{1, 0, 1, 0};
  ainternal::apply_step_5(cmc4x4);
  EXPECT_THAT(
    cmc4x4.cost_matrix,
    IsSame(
      Eigen::Matrix<double, 4, 4>{{0, 0, 0, 2}, {33, 0, 35, 0}, {0, 20, 33, 22},
        {0, 30, 43, 32}}));
}
