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
#include "path_planner.hpp"
#include "types/world.hpp"

#include <chrono>

#include <gtest/gtest.h>
#include <ateam_geometry/ateam_geometry.hpp>

namespace ateam_kenobi {
    /*
    Fixture for path test setup
    */
    class GetPathTest : public ::testing::Test {
        protected:
            path_planning::PathPlanner path_planner;
            World world;
            path_planning::PlannerOptions planner_options;
            std::vector<ateam_geometry::AnyShape> obstacles;
            std::chrono::duration<double> start_time;
            std::chrono::duration<double> allowed_extra_time = std::chrono::milliseconds(100);

            GetPathTest() {}
            virtual ~GetPathTest(){}
            virtual void SetUp() override {
                // Mock field definition
                // Should I move this?
                world.field.field_length = 0;
                world.field.field_width = 0;
                world.field.boundary_width = 0;
                world.ball.pos = ateam_geometry::Point(50, 50);
                start_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now());
            }
            virtual void TearDown(){
                obstacles.erase(obstacles.begin(), obstacles.end());
            }
    };

    TEST_F(GetPathTest, StraightPath){
        const auto start = ateam_geometry::Point(0, 0);
        const auto end = ateam_geometry::Point(1, 1);
        auto path = path_planner.getPath(
            start, end, world, obstacles, planner_options);
        const auto end_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now());
        EXPECT_NE(path, {});
        EXPECT_EQ(path, {start, end});
        EXPECT_EQ(path.size(), 2);
        EXPECT_LT(end_time - start_time, planner_options.search_time_limit + allowed_extra_time);
    }

    TEST_F(GetPathTest, PathWithSingleObstacle){
        const auto start = ateam_geometry::Point(0, 0);
        const auto end = ateam_geometry::Point(3, 3);
        const auto obstacle = ateam_geometry::makeCircle(ateam_geometry::Point(1, 1), 0.1);
        obstacles.push_back(obstacle);
        auto path = path_planner.getPath(
            start, end, world, obstacles, planner_options);
        const auto end_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now());
        EXPECT_NE(path, {});
        EXPECT_NE(path, {start, end});
        EXPECT_GT(path.size(), 2);
        EXPECT_LT(end_time - start_time, planner_options.search_time_limit + allowed_extra_time);
    }

    TEST_F(GetPathTest, PathWithMultipleObstacles){
        const auto start = ateam_geometry::Point(0, 0);
        const auto end = ateam_geometry::Point(3, 3);
        const auto obstacle_1 = ateam_geometry::makeCircle(ateam_geometry::Point(1, 1), 0.1);
        obstacles.push_back(obstacle_1);
        const auto obstacle_2 = ateam_geometry::makeCircle(ateam_geometry::Point(1, 2), 0.1);
        obstacles.push_back(obstacle_2);
        auto path = path_planner.getPath(
            start, end, world, obstacles, planner_options);
        const auto end_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now());
        EXPECT_NE(path, {});
        EXPECT_NE(path, {start, end});
        EXPECT_GT(path.size(), 2);
        EXPECT_LT(end_time - start_time, planner_options.search_time_limit + allowed_extra_time);
    }

    // Note: this will need a mock message/set of constants for the field boundaries
    TEST_F(GetPathTest, PathWithFieldBoundaries){}

    // We'll need a mock, use the same one as above
    TEST(CheckValidState, ValidState){}

    TEST(CheckValidState, OutOfBounds){}

    TEST(CheckValidState, Obstacle){}

    TEST(Obstacles, CreateObstaclesFromRobots){}

    TEST(Obstacles, GetDefaultObstacles){}
} // namespace ateam_kenobi