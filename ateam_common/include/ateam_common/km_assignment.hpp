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
#ifndef ATEAM_COMMON__KM_ASSIGNMENT_HPP_
#define ATEAM_COMMON__KM_ASSIGNMENT_HPP_

#include <Eigen/Dense>
#include <vector>

namespace ateam_common::assignment {
    const Eigen::MatrixXd make_square_cost_matrix(
        const Eigen::MatrixXd & cost_matrix);

    inline void compute_slack(
        const long x,
        std::vector<double>& slack,
        std::vector<double>& slackx,
        const Eigen::MatrixXd& cost,
        const std::vector<double>& lx,
        const std::vector<double>& ly
    );

    std::vector<long> max_cost_assignment (
        const Eigen::MatrixXd& cost_matrix
    );
}

#endif // ATEAM_COMMON__KM_ASSIGNMENT_HPP_