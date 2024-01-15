// Copyright 2024 A Team
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

#ifndef ATEAM_COMMON__BB_ASSIGNMENT_HPP_
#define ATEAM_COMMON__BB_ASSIGNMENT_HPP_

#include <vector>
#include "ateam_kenobi/types/robot.hpp"

namespace ateam_common::bb_assignment
{
    class BBSolver {
        public:
            BBSolver();
            ~BBSolver();

            std::vector<bool> getAssignments(std::vector<Robot> assignable_robots, int num_assignments);

        private:
            std::vector<Robot> assignable_robots;
            int max_cost;
            const int num_assignments;
            std::vector<bool> best_solution;
            std::vector<bool> current_solution;
            int num_currently_assigned;
            int current_cost;
            int future_assigned; // From BB
            int future_cost; // From BB
            int current_level; // of the tree
            int bb_level; // of the tree from BB
    }
} // namespace ateam_common::bb_assignment

#endif  // ATEAM_COMMON__ASSIGNMENT_HPP_