#include "EmbeddedRemeshing.hpp"

namespace wmtk::components::internal {
EmbeddedRemeshing2D::EmbeddedRemeshing2D(
    Eigen::MatrixXi& E_,
    Eigen::MatrixXd& V_,
    double blank_rate_,
    double resolute_area_)
    : E(E_)
    , V(V_)
    , blank_rate(blank_rate_)
    , resolute_area(resolute_area_)
{
    F.resize(0, 0);
    for (int i = 0; i < V.rows(); ++i) {
        markedV.push_back(i);
    }
    for (int i = 0; i < E.rows(); ++i) {
        markedE.push_back(std::pair<int, int>(E(i, 0), E(i, 1)));
    }
}
void EmbeddedRemeshing2D::compute_bounding_vaule(
    double& max_x,
    double& max_y,
    double& min_x,
    double& min_y)
{
    max_x = max_y = std::numeric_limits<double>::lowest();
    min_x = min_y = std::numeric_limits<double>::max();
    for (int i = 0; i < V.rows(); ++i) {
        max_x = std::max(max_x, V(i, 0));
        max_y = std::max(max_y, V(i, 1));
        min_x = std::min(max_x, V(i, 0));
        min_y = std::min(max_x, V(i, 1));
    }
    double blank_region_length_x = (max_x - min_x) * blank_rate;
    double blank_region_length_y = (max_y - min_y) * blank_rate;
    max_x += blank_region_length_x;
    min_x -= blank_region_length_x;
    max_y += blank_region_length_y;
    min_y -= blank_region_length_y;
}
void EmbeddedRemeshing2D::process()
{
    // find boundingbox
    double max_x, min_x, max_y, min_y;
    compute_bounding_vaule(max_x, max_y, min_x, min_y);

    Eigen::MatrixXd BoundingBoxV(4, 2);
    Eigen::MatrixXi BoundingBoxE(4, 2);
    BoundingBoxV << max_x, max_y, min_x, max_y, min_x, min_y, max_x, min_y;
    BoundingBoxE << markedV.size(), markedV.size() + 1, markedV.size() + 1, markedV.size() + 2,
        markedV.size() + 2, markedV.size() + 3, markedV.size() + 3, markedV.size();
    Eigen::MatrixXd tempV;
    Eigen::MatrixXi tempE;
    tempV = V + BoundingBoxV;
    tempE = E + BoundingBoxE;

    Eigen::MatrixXd H;
    H.resize(1, 2);
    H << max_x + 1.0, max_y + 1.0;

    // the fourth parameter is resolution, we should discuss it maybe.
    igl::triangle::triangulate(tempV, tempE, H, "a0.1q", V, F);

    // need to connect the topology
    // it would be easy, just check each edge and then move on
    std::vector<std::vector<int>> A;
    igl::adjacency_list(F, A);

    std::vector<std::pair<int, int>> tempEmarked;
    std::vector<int> tempVmarked;
    for (const auto& e : markedE) {
        // find marked E index before triangulate operation
        int vid0 = e.first, vid1 = e.second;
        Eigen::Vector2<double> p0, p1, dir;
        p0 << V(vid0, 0), V(vid0, 1);
        p1 << V(vid1, 0), V(vid1, 1);
        int cur_vid = vid0;
        tempVmarked.push_back(cur_vid);
        // our start vertex is p0, and constantly head to p1, and stop when encounter p1
        while (cur_vid != vid1) {
            Eigen::Vector2<double> cur_p;
            cur_p << V(cur_vid, 0), V(cur_vid, 1);
            dir = (p1 - cur_p).normalized();
            std::vector<int>& neighbour_v_list = A[cur_vid];
            double value_dir = -1;
            int next_vid = -1;
            // for each step, just foward the direction with less value of dot operation(sin)
            for (const auto neighbour_id : neighbour_v_list) {
                Eigen::Vector2<double> p, cur_dir;
                p << V(neighbour_id, 0), V(neighbour_id, 1);
                cur_dir = (p - cur_p).normalized();
                if (value_dir < cur_dir.dot(dir)) {
                    value_dir = cur_dir.dot(dir);
                    next_vid = neighbour_id;
                }
            }
            int vid0_ = cur_vid, vid1_ = next_vid;
            if (vid0_ > vid1_) std::swap(vid0_, vid1_);
            tempEmarked.push_back(std::pair<int, int>(vid0_, vid1_));
            tempVmarked.push_back(next_vid);
            cur_vid = next_vid;
        }
    }
    std::sort(tempVmarked.begin(), tempVmarked.end());
    tempVmarked.erase(std::unique(tempVmarked.begin(), tempVmarked.end()), tempVmarked.end());
    markedE = tempEmarked;
    markedV = tempVmarked;

    Vtags = std::vector<bool>(V.rows(), false);
    for (int i = 0; i < markedV.size(); ++i) {
        Vtags[markedV[i]] = true;
    }
}

} // namespace wmtk::components::internal