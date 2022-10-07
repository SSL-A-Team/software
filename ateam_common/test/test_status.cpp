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
#include <string>
#include "ateam_common/status.hpp"

ateam::Status AlwaysFailStatus()
{
  return ateam::Failure("Fail string");
}

ateam::Status AlwaysPassStatus()
{
  return ateam::Ok();
}

ateam::StatusOr<int> AlwaysFailInt()
{
  return ateam::Failure<int>("Fail string");
}

ateam::StatusOr<int> Always10Int()
{
  return ateam::Ok(10);
}

TEST(Status, assign_or_throw_bad)
{
  EXPECT_THROW(ATEAM_ASSIGN_OR_THROW(auto a, AlwaysFailInt(), "Oops failed1"), std::string);
  EXPECT_THROW(ATEAM_ASSIGN_OR_THROW(auto b, AlwaysFailInt(), "Oops failed2"), std::string);
}

TEST(Status, assign_or_throw_good)
{
  ATEAM_ASSIGN_OR_THROW(auto a, Always10Int(), "Oops failed");
  EXPECT_EQ(a, 10);
  ATEAM_ASSIGN_OR_THROW(auto b, Always10Int(), "Oops failed");
  EXPECT_EQ(b, 10);
}

TEST(Status, always)
{
  EXPECT_NO_THROW(ATEAM_CHECK(AlwaysPassStatus(), "Oops failed1"));
  EXPECT_THROW(ATEAM_CHECK(AlwaysFailStatus(), "Oops failed1"), std::string);
}
