#pragma once

#include <nanoflann.hpp>
#include <wmtk/Types.hpp>

namespace wmtk {

struct PointCloud
{
    using coord_t = double; //!< The type of each coordinate

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const { return pts[idx][dim]; }

    std::vector<Vector3d> pts;

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const
    {
        return false;
    }
};

class KNN
{
    // construct a kd-tree index:
    using my_kd_tree_t = nanoflann::
        KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud>, PointCloud, 3>;

public:
    KNN(const std::vector<Vector3d>& pts);

    void nearest_neighbor(
        const Vector3d& query_point,
        uint32_t& nearest_point_index,
        double& sq_dist) const;

    void nearest_neighbors(
        const Vector3d& query_point,
        std::vector<uint32_t>& nearest_point_indices,
        std::vector<double>& sq_dist) const;

    const Vector3d& point(const uint32_t& index) const { return point_cloud.pts[index]; }

private:
    PointCloud point_cloud;
    std::unique_ptr<my_kd_tree_t> m_kd_tree;
};

} // namespace wmtk