#include <vector>
#include <limits>


#include <unordered_set>

namespace ateam_common::assignment
{

/**
 * Sets a = min(a, b)
 * @return true if b < a
 */
template <class T>
bool ckmin(T &a, const T &b) { return b < a ? a = b, 1 : 0; }


// wow even the copy pasted code is segfaulting amazing
// Copy pasted because I dont care about anything anymore
template <class T>
std::unordered_map<std::size_t, std::size_t> johnsons_hungarian(const Eigen::Matrix<T,  Eigen::Dynamic,  Eigen::Dynamic> &C) {
    const size_t J = static_cast<size_t>(C.rows());
    const size_t W = static_cast<size_t>(C.cols());
    // if its not you shouldnt be using this but someone is going to use this directly on the robot count and its going to break at some point so...
    // assert(J <= W);
    // job[w] = job assigned to w-th worker, or -1 if no job assigned
    // note: a W-th worker was added for convenience
    std::vector<int> job(W + 1, -1);
    std::vector<T> h(W);  // Johnson potentials
    std::unordered_map<std::size_t, std::size_t> answers;
    T ans_cur = 0;
    const T inf = std::numeric_limits<T>::max();
    // assign j_cur-th job using Dijkstra with potentials
    for (int j_cur = 0; j_cur < J; ++j_cur) {
        int w_cur = W;  // unvisited worker with minimum distance
        job.at(w_cur) = j_cur;
        std::vector<T> dist(W + 1, inf);  // Johnson-reduced distances
        dist[W] = 0;
        std::vector<bool> vis(W + 1);     // whether visited yet
        std::vector<size_t> prv(W + 1, -1);  // previous worker on shortest path
        while (job[w_cur] != -1) {   // Dijkstra step: pop min worker from heap
            T min_dist = inf;
            vis[w_cur] = true;
            int w_next = -1;  // next unvisited worker with minimum distance
            // consider extending shortest path by w_cur -> job[w_cur] -> w
            for (int w = 0; w < W; ++w) {
                if (!vis[w]) {
                    // sum of reduced edge weights w_cur -> job[w_cur] -> w
                    T edge = C(job[w_cur], w) - h[w];
                    if (w_cur != W) {
                        edge -= C(job[w_cur], w_cur) - h[w_cur];
                        assert(edge >= 0);  // consequence of Johnson potentials
                    }
                    if (ckmin(dist[w], dist[w_cur] + edge)) prv[w] = w_cur;
                    if (ckmin(min_dist, dist[w])) w_next = w;
                }
            }
            w_cur = w_next;
        }
        for (int w = 0; w < W; ++w) {  // update potentials
            ckmin(dist[w], dist[w_cur]);
            h[w] += dist[w];
        }
        ans_cur += h[w_cur];
        for (int w; w_cur != W; w_cur = w) {
            job[w_cur] = job[w = prv[w_cur]];
        }
        answers.emplace(std::make_pair(j_cur, ans_cur));
    }
    return answers; // size of jobs the assigned worker
}
}