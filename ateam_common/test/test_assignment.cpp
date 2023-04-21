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

MATCHER_P(IsSameSize, m, "") {
  *result_listener << "where the size (" << arg.rows() << ", " << arg.cols() << ") ";
  *result_listener << "!= expected (" << m.rows() << ", " << m.cols() << ")";
  return arg.rows() == m.rows() && arg.cols() == m.cols();
}

MATCHER_P(HasOriginalData, original, "") {
  // The original data should stay the same
  *result_listener << "where the new data:\n\r" << arg << "\n\r";
  *result_listener << "is not the original data:\n\r" << original << "\n\r";
  return (arg.block(0, 0, original.rows(), original.cols()).array() - original.array()).matrix().sum() < 1e-6;
}

MATCHER_P(HasUnfilledData, original, "") {
  *result_listener << "where the unfilled data:\n\r" << arg << "\n\r";
  *result_listener << "is incorrect compared to original data:\n\r" << original << "\n\r";
  if (arg.size() == original.size()) {
    return true;
  }

  // Average cell value in the non-original cells should be UNFILLED_LARGE_VALUE
  Eigen::MatrixXd temp = arg;
  std::size_t diff = arg.size() - original.size();
  temp.block(0, 0, original.rows(), original.cols()) = Eigen::MatrixXd::Zero(original.rows(), original.cols());
  return temp.sum() / diff == ainternal::UNFILLED_LARGE_VALUE;
}

TEST(Assignment, SquareizeMatrix)
{
  // No-op with square input
  Eigen::Matrix<double, 1, 1> square1x1{{1}};
  EXPECT_THAT(ainternal::SquarizeMatrix(square1x1), IsSameSize(square1x1));
  EXPECT_THAT(ainternal::SquarizeMatrix(square1x1), HasOriginalData(square1x1));
  EXPECT_THAT(ainternal::SquarizeMatrix(square1x1), HasUnfilledData(square1x1));

  Eigen::Matrix2d square2x2{{1, 2}, {3, 4}};
  EXPECT_THAT(ainternal::SquarizeMatrix(square2x2), IsSameSize(square2x2));
  EXPECT_THAT(ainternal::SquarizeMatrix(square2x2), HasOriginalData(square2x2));
  EXPECT_THAT(ainternal::SquarizeMatrix(square2x2), HasUnfilledData(square2x2));

  Eigen::Matrix3d square3x3{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  EXPECT_THAT(ainternal::SquarizeMatrix(square3x3), IsSameSize(square3x3));
  EXPECT_THAT(ainternal::SquarizeMatrix(square3x3), HasOriginalData(square3x3));
  EXPECT_THAT(ainternal::SquarizeMatrix(square3x3), HasUnfilledData(square3x3));

  // Too many rows
  Eigen::Matrix<double, 2, 1> row2x1{{1}, {2}};
  EXPECT_THAT(ainternal::SquarizeMatrix(row2x1), IsSameSize(square2x2));
  EXPECT_THAT(ainternal::SquarizeMatrix(row2x1), HasOriginalData(row2x1));
  EXPECT_THAT(ainternal::SquarizeMatrix(row2x1), HasUnfilledData(row2x1));

  Eigen::Matrix<double, 3, 1> row3x1{{1}, {2}, {3}};
  EXPECT_THAT(ainternal::SquarizeMatrix(row3x1), IsSameSize(square3x3));
  EXPECT_THAT(ainternal::SquarizeMatrix(row3x1), HasOriginalData(row3x1));
  EXPECT_THAT(ainternal::SquarizeMatrix(row3x1), HasUnfilledData(row3x1));

  // Too many cols
  Eigen::Matrix<double, 2, 1> col1x2{{1, 2}};
  EXPECT_THAT(ainternal::SquarizeMatrix(col1x2), IsSameSize(square2x2));
  EXPECT_THAT(ainternal::SquarizeMatrix(col1x2), HasOriginalData(col1x2));
  EXPECT_THAT(ainternal::SquarizeMatrix(col1x2), HasUnfilledData(col1x2));

  Eigen::Matrix<double, 3, 1> col1x3{{1, 2, 3}};
  EXPECT_THAT(ainternal::SquarizeMatrix(col1x3), IsSameSize(square3x3));
  EXPECT_THAT(ainternal::SquarizeMatrix(col1x3), HasOriginalData(col1x3));
  EXPECT_THAT(ainternal::SquarizeMatrix(col1x3), HasUnfilledData(col1x3));
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

TEST(Assignment, ApplyStep1)
{
  Eigen::Matrix<double, 1, 1> square1x1{{1}};
  EXPECT_THAT(ainternal::ApplyStep1(square1x1), Has0ReferencedRows(square1x1));

  Eigen::Matrix2d square2x2{{1, 2}, {3, 4}};
  EXPECT_THAT(ainternal::ApplyStep1(square2x2), Has0ReferencedRows(square2x2));

  Eigen::Matrix3d square3x3{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  EXPECT_THAT(ainternal::ApplyStep1(square3x3), Has0ReferencedRows(square3x3));
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

TEST(Assignment, ApplyStep2)
{
  Eigen::Matrix<double, 1, 1> square1x1{{1}};
  EXPECT_THAT(ainternal::ApplyStep2(square1x1), Has0ReferencedCols(square1x1));

  Eigen::Matrix2d square2x2{{1, 2}, {3, 4}};
  EXPECT_THAT(ainternal::ApplyStep2(square2x2), Has0ReferencedCols(square2x2));

  Eigen::Matrix3d square3x3{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  EXPECT_THAT(ainternal::ApplyStep2(square3x3), Has0ReferencedCols(square3x3));
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

TEST(Assignment, ApplyStep3)
{
  // Straightforward marking
  Eigen::Matrix<double, 1, 1> square1x1{{0}};
  Eigen::Matrix<double, 1, 1> mark1x1{{1}};
  EXPECT_THAT(ainternal::ApplyStep3(square1x1), IsSame(mark1x1));

  Eigen::Matrix2d square2x2{{0, 2}, {3, 0}};
  Eigen::Matrix2d mark2x2{{1, 0}, {0, 1}};
  EXPECT_THAT(ainternal::ApplyStep3(square2x2), IsSame(mark2x2));

  Eigen::Matrix3d square3x3{{1, 0, 3}, {0, 5, 6}, {7, 8, 0}};
  Eigen::Matrix3d mark3x3{{0, 1, 0}, {1, 0, 0}, {0, 0, 1}};
  EXPECT_THAT(ainternal::ApplyStep3(square3x3), IsSame(mark3x3));

  // More complicated marking
  Eigen::Matrix2d zero2x2{{0, 0}, {0, 0}};
  Eigen::Matrix2d zmark2x2{{1, 0}, {0, 1}};
  EXPECT_THAT(ainternal::ApplyStep3(zero2x2), IsSame(zmark2x2));

  Eigen::Matrix2d inv2x2{{1, 0}, {0, 1}};
  Eigen::Matrix2d imark2x2{{0, 1}, {1, 0}};
  EXPECT_THAT(ainternal::ApplyStep3(inv2x2), IsSame(imark2x2));
}

TEST(Assignment, HasUniqueAssignments)
{
  // Valid
  Eigen::Matrix<int, 1, 1> valid1x1{{1}};
  EXPECT_TRUE(ainternal::HasUniqueAssignments(valid1x1));
  Eigen::Matrix2i valid2x2_1{{1, 0}, {0, 1}};
  EXPECT_TRUE(ainternal::HasUniqueAssignments(valid2x2_1));
  Eigen::Matrix2i valid2x2_2{{0, 1}, {1, 0}};
  EXPECT_TRUE(ainternal::HasUniqueAssignments(valid2x2_2));
  Eigen::Matrix3i valid3x3{{0, 1, 0}, {1, 0, 0}, {0, 0, 1}};
  EXPECT_TRUE(ainternal::HasUniqueAssignments(valid3x3));

  // Invalid
  Eigen::Matrix<int, 1, 1> invalid1x1{{0}};
  EXPECT_FALSE(ainternal::HasUniqueAssignments(invalid1x1));
  Eigen::Matrix2i invalid2x2_1{{1, 1}, {0, 1}};
  EXPECT_FALSE(ainternal::HasUniqueAssignments(invalid2x2_1));
  Eigen::Matrix2i invalid2x2_2{{1, 1}, {1, 0}};
  EXPECT_FALSE(ainternal::HasUniqueAssignments(invalid2x2_2));
  Eigen::Matrix2i invalid2x2_3{{1, 1}, {0, 0}};
  EXPECT_FALSE(ainternal::HasUniqueAssignments(invalid2x2_3));
  Eigen::Matrix2i invalid2x2_4{{1, 0}, {1, 0}};
  EXPECT_FALSE(ainternal::HasUniqueAssignments(invalid2x2_4));
  Eigen::Matrix2i invalid2x2_5{{1, 0}, {0, 0}};
  EXPECT_FALSE(ainternal::HasUniqueAssignments(invalid2x2_5));
  Eigen::Matrix2i invalid2x2_6{{0, 1}, {0, 1}};
  EXPECT_FALSE(ainternal::HasUniqueAssignments(invalid2x2_6));
  Eigen::Matrix2i invalid2x2_7{{0, 1}, {0, 0}};
  EXPECT_FALSE(ainternal::HasUniqueAssignments(invalid2x2_7));
  Eigen::Matrix2i invalid2x2_8{{0, 0}, {0, 1}};
  EXPECT_FALSE(ainternal::HasUniqueAssignments(invalid2x2_8));
  Eigen::Matrix2i invalid2x2_9{{0, 0}, {1, 1}};
  EXPECT_FALSE(ainternal::HasUniqueAssignments(invalid2x2_9));
  Eigen::Matrix2i invalid2x2_10{{0, 0}, {1, 0}};
  EXPECT_FALSE(ainternal::HasUniqueAssignments(invalid2x2_10));
  Eigen::Matrix2i invalid2x2_11{{0, 0}, {0, 0}};
  EXPECT_FALSE(ainternal::HasUniqueAssignments(invalid2x2_11));
  Eigen::Matrix3i invalid3x3{{0, 1, 0}, {1, 1, 0}, {0, 0, 1}};
  EXPECT_FALSE(ainternal::HasUniqueAssignments(invalid3x3));
}