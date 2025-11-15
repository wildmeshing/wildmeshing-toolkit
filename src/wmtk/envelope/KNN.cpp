#include "KNN.hpp"

namespace wmtk {

KNN::KNN(const std::vector<Vector3d>& pts)
{
    point_cloud.pts = pts;
    m_kd_tree = std::make_unique<my_kd_tree_t>(3, point_cloud);
}

void KNN::nearest_neighbor(const Vector3d& query_point, uint32_t& nearest_point, double& sq_dist)
    const
{
    m_kd_tree->knnSearch(query_point.data(), 1, &nearest_point, &sq_dist);
}

void KNN::nearest_neighbors(
    const Vector3d& query_point,
    std::vector<uint32_t>& nearest_points,
    std::vector<double>& sq_dist) const
{
    assert(nearest_points.size() == sq_dist.size());

    m_kd_tree->knnSearch(
        query_point.data(),
        nearest_points.size(),
        nearest_points.data(),
        sq_dist.data());
}

} // namespace wmtk