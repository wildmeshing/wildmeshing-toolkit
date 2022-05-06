#pragma once

#include <Eigen/Core>
#include <memory>
#include "fastenvelope/FastEnvelope.h"

namespace GEO {
class MeshFacetsAABBWithEps;
class Mesh;
} // namespace GEO

namespace sample_envelope {
class SampleEnvelope
{
public:
    SampleEnvelope(bool exact = false)
        : use_exact(exact){};
    double eps2 = 1e-6;
    double sampling_dist = 1e-3;
    bool use_exact = false;
    void init(
        const std::vector<Eigen::Vector3d>& m_ver,
        const std::vector<Eigen::Vector3i>& m_faces,
        const double);
    bool is_outside(const std::array<Eigen::Vector3d, 3>& tris);
    bool is_outside(const Eigen::Vector3d& pts);

private:
    std::shared_ptr<GEO::MeshFacetsAABBWithEps> geo_tree_ptr_;
    std::shared_ptr<GEO::Mesh> geo_polyhedron_ptr_;
    std::vector<int> geo_vertex_ind;
    std::vector<int> geo_face_ind;
private:
    fastEnvelope::FastEnvelope exact_envelope;
};
} // namespace sample_envelope