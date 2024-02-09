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
#ifndef ATEAM_COMMON__SIMPLE_ASSIGNMENT_HPP_
#define ATEAM_COMMON__SIMPLE_ASSIGNMENT_HPP_

#include <Eigen/Dense>

namespace ateam_common::assignment {
    const Eigen::MatrixXd make_square_cost_matrix(
        const Eigen::MatrixXd & cost_matrix);

    const 
}

#include <vector>
#include <deque>

inline void compute_slack(
    const long x,
    std::vector<double>& slack,
    std::vector<double>& slackx,
    const Eigen::MatrixXd& cost,
    const std::vector<double>& lx,
    const std::vector<double>& ly
)
{
    for (long y = 0; y < static_cast<int>(cost.cols()); ++y)
    {
        if (lx[x] + ly[y] - cost(x,y) < slack[y])
        {
            slack[y] = lx[x] + ly[y] - cost(x,y);
            slackx[y] = x;
        }
    }
}

// ----------------------------------------------------------------------------------------

std::vector<long> max_cost_assignment (
    const Eigen::MatrixXd& cost_matrix
)                         
{
    /*
    
    The dlib implementation of this algorithm is based on info available below...
    although the links were broken when I tried them.
        http://www.math.uwo.ca/~mdawes/courses/344/kuhn-munkres.pdf
        http://www.topcoder.com/tc?module=Static&d1=tutorials&d2=hungarianAlgorithm

    Note that this is the fast O(n^3) version of the algorithm.
    */

    std::vector<double> lx, ly;
    std::vector<long> xy;
    std::vector<long> yx;
    std::vector<char> S, T;
    std::vector<double> slack;
    std::vector<double> slackx;
    std::vector<long> aug_path;

    size_t cost_size = static_cast<size_t>(cost_matrix.cols());

    // Initially, nothing is matched. 
    xy.assign(cost_size, -1);
    yx.assign(cost_size, -1);
    /*
    We maintain the following invariant:
        Vertex x is matched to vertex xy[x] and
        vertex y is matched to vertex yx[y].

        A value of -1 means a vertex isn't matched to anything.  Moreover,
        x corresponds to rows of the cost matrix and y corresponds to the
        columns of the cost matrix.  So we are matching X to Y.
    */


    // Create an initial feasible labeling.  Moreover, in the following
    // code we will always have: 
    //     for all valid x and y:  lx[x] + ly[y] >= cost(x,y)
    //lx.resize(cost_size);

    ly.assign(cost_size,0);
    // for (size_t x = 0; x < cost_size; ++x)
    //     lx[x] = max(rowm(cost,x));
    // The below should be equivalent to the other lx lines
    Eigen::VectorXd lx = cost_matrix.rowwise().maxCoeff();
    // Now grow the match set by picking edges from the equality subgraph until
    // we have a complete matching.
    for (long match_size = 0; match_size < cost_size; ++match_size)
    {
        std::deque<long> q;

        // Empty out the S and T sets
        S.assign(cost_size, false);
        T.assign(cost_size, false);

        // clear out old slack values
        slack.assign(cost_size, std::numeric_limits<double>::max());
        slackx.resize(cost_size);
        /*
            slack and slackx are maintained such that we always
            have the following (once they get initialized by compute_slack() below):
                - for all y:
                    - let x == slackx[y]
                    - slack[y] == lx[x] + ly[y] - cost(x,y)
        */

        aug_path.assign(cost_size, -1);

        for (long x = 0; x < cost_size; ++x)
        {
            // If x is not matched to anything
            if (xy[x] == -1)
            {
                q.push_back(x);
                S[x] = true;

                compute_slack(x, slack, slackx, cost_matrix, lx, ly);
                break;
            }
        }


        long x_start = 0;
        long y_start = 0;

        // Find an augmenting path.  
        bool found_augmenting_path = false;
        while (!found_augmenting_path)
        {
            while (q.size() > 0 && !found_augmenting_path)
            {
                const long x = q.front();
                q.pop_front();
                for (long y = 0; y < cost_size; ++y)
                {
                    if (cost(x,y) == lx[x] + ly[y] && !T[y])
                    {
                        // if vertex y isn't matched with anything
                        if (yx[y] == -1) 
                        {
                            y_start = y;
                            x_start = x;
                            found_augmenting_path = true;
                            break;
                        }

                        T[y] = true;
                        q.push_back(yx[y]);

                        aug_path[yx[y]] = x;
                        S[yx[y]] = true;
                        compute_slack(yx[y], slack, slackx, cost, lx, ly);
                    }
                }
            }

            if (found_augmenting_path)
                break;


            // Since we didn't find an augmenting path we need to improve the 
            // feasible labeling stored in lx and ly.  We also need to keep the
            // slack updated accordingly.
            double delta = std::numeric_limits<double>::max();
            for (unsigned long i = 0; i < T.size(); ++i)
            {
                if (!T[i])
                    delta = std::min(delta, slack[i]);
            }
            for (unsigned long i = 0; i < T.size(); ++i)
            {
                if (S[i])
                    lx[i] -= delta;

                if (T[i])
                    ly[i] += delta;
                else
                    slack[i] -= delta;
            }



            q.clear();
            for (long y = 0; y < cost_size; ++y)
            {
                if (!T[y] && slack[y] == 0)
                {
                    // if vertex y isn't matched with anything
                    if (yx[y] == -1)
                    {
                        x_start = slackx[y];
                        y_start = y;
                        found_augmenting_path = true;
                        break;
                    }
                    else
                    {
                        T[y] = true;
                        if (!S[yx[y]])
                        {
                            q.push_back(yx[y]);

                            aug_path[yx[y]] = slackx[y];
                            S[yx[y]] = true;
                            compute_slack(yx[y], slack, slackx, cost, lx, ly);
                        }
                    }
                }
            }
        } // end while (!found_augmenting_path)

        // Flip the edges along the augmenting path.  This means we will add one more
        // item to our matching.
        for (long cx = x_start, cy = y_start, ty; 
                cx != -1; 
                cx = aug_path[cx], cy = ty)
        {
            ty = xy[cx];
            yx[cy] = cx;
            xy[cx] = cy;
        }

    }


    return xy;
}

// ----------------------------------------------------------------------------------------

}

#endif // ATEAM_COMMON__SIMPLE_ASSIGNMENT_HPP_