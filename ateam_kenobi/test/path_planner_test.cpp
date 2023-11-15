// Copyright 2023 A Team
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
#include "path_planner.hpp"

// TODO - Is there a way to use a single shared path planner for all these?
// I.e. through a set up function?

TEST(GetPaths, StraightPath){}

TEST(GetPaths, PathWithSingleObstacle){}

TEST(GetPaths, PathWithMultipleObstacles){}

// Note: this will need a mock message/set of constants for the field boundaries
TEST(GetPaths, PathWithFieldBoundaries){}

// We'll need a mock, use the same one as above
TEST(CheckValidState, ValidState){}

TEST(CheckValidState, OutOfBounds){}

TEST(CheckValidState, Obstacle){}

TEST(Obstacles, CreateObstaclesFromRobots){}

TEST(Obstacles, GetDefaultObstacles){}