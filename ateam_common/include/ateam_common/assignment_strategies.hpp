#include <vector>
#include <limits>
#include <cassert>
#include <iostream>
#include <unordered_set>

namespace ateam_common::assignment
{

class ford_fulkerson {
  public:

    int cost[31][31]; //cost matrix
    int n, max_match; //n workers and n jobs
    int lx[31], ly[31]; //labels of X and Y parts
    int xy[31]; //xy[x] - vertex that is matched with x,
    int yx[31]; //yx[y] - vertex that is matched with y
    bool S[31], T[31]; //sets S and T in algorithm
    int slack[31]; //as in the algorithm description
    int slackx[31]; //slackx[y] such a vertex, that
    int prev_ious[31]; //array for memorizing alternating p

    void init_labels()
    {
        memset(lx, 0, sizeof(lx));
        memset(ly, 0, sizeof(ly));
        for (int x = 0; x < n; x++)
        for (int y = 0; y < n; y++)
        lx[x] = max(lx[x], cost[x][y]);
    }

    void update_labels()
    {
        int x, y;
        int delta = 99999999; //init delta as infinity
        for (y = 0; y < n; y++) //calculate delta using slack
            if (!T[y])
                delta = min(delta, slack[y]);
        for (x = 0; x < n; x++) //update X labels
            if (S[x])
                lx[x] -= delta;
        for (y = 0; y < n; y++) //update Y labels
            if (T[y])
                ly[y] += delta;
        for (y = 0; y < n; y++) //update slack array
            if (!T[y])
                slack[y] -= delta;
    }

    void add_to_tree(int x, int prev_iousx)
    //x - current vertex,prev_iousx - vertex from X before x in the alternating path,
    //so we add edges (prev_iousx, xy[x]), (xy[x], x)
    {
        S[x] = true; //add x to S
        prev_ious[x] = prev_iousx; //we need this when augmenting
        for (int y = 0; y < n; y++) //update slacks, because we add new vertex to S
            if (lx[x] + ly[y] - cost[x][y] < slack[y])
            {
                slack[y] = lx[x] + ly[y] - cost[x][y];
                slackx[y] = x;
            }
    }

    void augment() //main function of the algorithm
    {
        if (max_match == n) return; //check whether matching is already perfect
        int x, y, root; //just counters and root vertex
        int q[31], wr = 0, rd = 0; //q - queue for bfs, wr,rd - write and read
        //pos in queue
        memset(S, false, sizeof(S)); //init set S
        memset(T, false, sizeof(T)); //init set T
        memset(prev_ious, -1, sizeof(prev_ious)); //init set prev_ious - for the alternating tree

        for (x = 0; x < n; x++) //finding root of the tree
        {
            if (xy[x] == -1)
            {
                q[wr++] = root = x;
                prev_ious[x] = -2;
                S[x] = true;
                break;
            }
        }

        for (y = 0; y < n; y++) //initializing slack array
        {
            slack[y] = lx[root] + ly[y] - cost[root][y];
            slackx[y] = root;
        }

        //second part of augment() function
        while (true) //main cycle
        {
            while (rd < wr) //building tree with bfs cycle
            {
                x = q[rd++]; //current vertex from X part
                for (y = 0; y < n; y++) //iterate through all edges in equality graph
                    if (cost[x][y] == lx[x] + ly[y] && !T[y])
                    {
                        if (yx[y] == -1) break; //an exposed vertex in Y found, so
                                                //augmenting path exists!
                            T[y] = true; //else just add y to T,
                        q[wr++] = yx[y]; //add vertex yx[y], which is matched
                        //with y, to the queue
                        add_to_tree(yx[y], x); //add edges (x,y) and (y,yx[y]) to the tree
                    }
                if (y < n)
                    break; //augmenting path found!
            }
            if (y < n)
                break; //augmenting path found!

            update_labels(); //augmenting path not found, so improve labeling

            wr = rd = 0;
            for (y = 0; y < n; y++)
            //in this cycle we add edges that were added to the equality graph as a
            //result of improving the labeling, we add edge (slackx[y], y) to the tree if
            //and only if !T[y] && slack[y] == 0, also with this edge we add another one
            //(y, yx[y]) or augment the matching, if y was exposed
            if (!T[y] && slack[y] == 0)
            {
                if (yx[y] == -1) //exposed vertex in Y found - augmenting path exists!
                {
                    x = slackx[y];
                    break;
                }
                else
                {
                    T[y] = true; //else just add y to T,
                    if (!S[yx[y]])
                    {
                        q[wr++] = yx[y]; //add vertex yx[y], which is matched with
                        //y, to the queue
                        add_to_tree(yx[y], slackx[y]); //and add edges (x,y) and (y,
                        //yx[y]) to the tree
                    }
                }
            }
            if (y < n) break; //augmenting path found!
        }

        if (y < n) //we found augmenting path!
        {
            max_match++; //increment matching
            //in this cycle we inverse edges along augmenting path
            for (int cx = x, cy = y, ty; cx != -2; cx = prev_ious[cx], cy = ty)
            {
                ty = xy[cx];
                yx[cy] = cx;
                xy[cx] = cy;
            }
            augment(); //recall function, go to step 1 of the algorithm
        }
    }//end of augment() function

    int hungarian()
    {
        int ret = 0; //weight of the optimal matching
        max_match = 0; //number of vertices in current matching
        memset(xy, -1, sizeof(xy));
        memset(yx, -1, sizeof(yx));
        init_labels(); //step 0
        augment(); //steps 1-3

        for (int x = 0; x < n; x++) //forming answer there
            ret += cost[x][xy[x]];

        return ret;
    }

    int assignmentProblem(int Arr[], int N) {
        n = N;
        for(int i=0; i<n; i++)
            for(int j=0; j<n; j++)
                cost[i][j] = -1*Arr[i*n+j];

        int ans = -1 * hungarian();

        return ans;
    }
};









/**
 * Sets a = min(a, b)
 * @return true if b < a
 */
template <class T>
bool ckmin(T &a, const T &b) { return b < a ? a = b, 1 : 0; }

/**
 * Given J jobs and W workers (J <= W), computes the minimum cost to assign each
 * prefix of jobs to distinct workers.
 *
 * @tparam T a type large enough to represent integers on the order of J *
 * max(|C|)
 * @param C a matrix of dimensions JxW such that C[j][w] = cost to assign j-th
 * job to w-th worker (possibly negative)
 *
 * @return a vector of length J, with the j-th entry equaling the minimum cost
 * to assign the first (j+1) jobs to distinct workers
 */
template <class T>
std::unordered_map<std::size_t, std::size_t> hungarian(const Eigen::Matrix<T,  Eigen::Dynamic,  Eigen::Dynamic> &C) {

    const int J = (C.rows());
    const int W = (C.cols());
    assert(J <= W);
    // job[w] = job assigned to w-th worker, or -1 if no job assigned
    // note: a W-th worker was added for convenience
    std::vector<int> job(W + 1, -1);
    std::vector<T> ys(J), yt(W + 1);  // potentials
    // -yt[W] will equal the sum of all deltas
    std::unordered_map<std::size_t, std::size_t> answers;
    const T inf = std::numeric_limits<T>::max();
    for (int j_cur = 0; j_cur < J; ++j_cur) {  // assign j_cur-th job
        int w_cur = W;
        job[w_cur] = j_cur;
        // min reduced cost over edges from Z to worker w
        std::vector<T> min_to(W + 1, inf);
        std::vector<int> prv(W + 1, -1);  // previous worker on alternating path
        std::vector<bool> in_Z(W + 1);    // whether worker is in Z
        while (job[w_cur] != -1) {   // runs at most j_cur + 1 times
            in_Z[w_cur] = true;
            const int j = job[w_cur];
            T delta = inf;
            int w_next;
            for (int w = 0; w < W; ++w) {
                if (!in_Z[w]) {
                    if (ckmin(min_to[w], C(j, w) - ys[j] - yt[w]))
                        prv[w] = w_cur;
                    if (ckmin(delta, min_to[w])) w_next = w;
                }
            }
            // delta will always be non-negative,
            // except possibly during the first time this loop runs
            // if any entries of C[j_cur] are negative
            for (int w = 0; w <= W; ++w) {
                if (in_Z[w]) ys[job[w]] += delta, yt[w] -= delta;
                else min_to[w] -= delta;
            }
            w_cur = w_next;
        }
        // update assignments along alternating path
        for (int w; w_cur != W; w_cur = w) job[w_cur] = job[w = prv[w_cur]];
        answers.emplace(std::make_pair(j_cur, -yt[W]));
    }
    return answers;
}

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