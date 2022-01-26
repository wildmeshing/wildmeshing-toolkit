#pragma once

#include <BVH.hpp>

namespace wmtk {
class SamplingEnvelope
{
private:
    BVH::BVH m_bvh;
    double m_sampling_dist;
    double m_eps;
    double m_tol;

    Eigen::MatrixXd m_V;
    Eigen::MatrixXi m_F;

private:
    void sample_tri(
        const std::array<Eigen::Vector3d, 3>& tri,
        std::vector<Eigen::Vector3d>& samples) const;
    double point_tri_distance(const std::array<Eigen::Vector3d, 3>& tri, const Eigen::Vector3d& p)
        const;

public:
    SamplingEnvelope(double sampling_dist, double box_tolerance = 0);

    void init(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, double eps);
    bool is_outside(const std::array<Eigen::Vector3d, 3>& tri) const;
};
} // namespace wmtk
